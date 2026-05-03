/*
 * control.c — Simple wall-follow racing loop.
 *
 * Ports roborace_car/roborace_car.ino work() to Zephyr.
 * No IMU, no stuck/wrong-dir recovery. PID speed loop via car.c.
 *
 * All 6 sensors are used. Physical layout:
 *
 *          FRONT
 *   FL(4)       FR(1)
 * HL(5)           HR(0)
 *   L(3)         R(2)
 *          REAR
 *
 * Front obstacle : FL(4), FR(1)            — blocked-lane detection
 * Side wall dist : MIN(L,HL), MIN(R,HR)    — catches angled wall approaches
 */

#include "control.h"
#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "battery.h"
#include "wifi_cmd.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define CONTROL_STACK_SIZE 4096
#define CONTROL_PRIORITY   2

static K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_td;

static volatile bool  running;
static volatile bool  drv_enabled;
static volatile bool  manual_active;   /* set by $DRV; expires after 500 ms */
static volatile int   manual_steer;
static volatile float manual_speed;
static volatile int64_t last_drv_ms;

extern void wdt_feed_kick(void);

static int run_div;
static uint32_t telem_count;

static void send_telem(const int *s, int steer, float spd_target)
{
	float bat = battery_voltage();

	wifi_cmd_printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.2f,%.1f,%.2f\n",
			k_uptime_get(),
			s[0], s[1], s[2], s[3], s[4], s[5],
			steer, (double)taho_get_speed(), (double)spd_target,
			(double)bat);

	if ((++telem_count % 25) == 0) {  /* every ~5 s */
		LOG_INF("telem tx: %u frames", telem_count);
	}

	if (battery_is_low()) {
		wifi_cmd_printf("$WARN:BAT_LOW,v=%.2f\n", (double)bat);
	}
}

static void work(const struct car_settings *c)
{
	int *s = sensors_poll();

	int HR = s[IDX_HARD_RIGHT];
	int FR = s[IDX_FRONT_RIGHT];
	int R  = s[IDX_RIGHT];
	int L  = s[IDX_LEFT];
	int FL = s[IDX_FRONT_LEFT];
	int HL = s[IDX_HARD_LEFT];

	/* Front obstacle detection: diagonal sensors only */
	bool f_l = FL < c->front_obstacle_dist;
	bool f_r = FR < c->front_obstacle_dist;

	/* Side wall distances.
	 * 6-sensor mode: MIN of each pair catches angled approaches early.
	 * 4-sensor mode: pure side sensors only (legacy behaviour). */
	int left_wall  = c->use_six_sensors ? MIN(L, HL) : L;
	int right_wall = c->use_six_sensors ? MIN(R, HR) : R;

	int diff;
	if (left_wall > c->side_open_dist && right_wall > c->side_open_dist) {
		/* Both sides open: bias right to find a wall */
		diff = 800;
	} else {
		diff = right_wall - left_wall;
	}

	/* Boxed in: core 4 sensors always checked; HL/HR added in 6-sensor mode */
	bool boxed = (L  < c->all_close_dist) && (FL < c->all_close_dist) &&
		     (FR < c->all_close_dist) && (R  < c->all_close_dist) &&
		     (!c->use_six_sensors ||
		      (HL < c->all_close_dist && HR < c->all_close_dist));
	if (boxed) {
		diff = 800;
	}

	int how_clear = (int)f_l + (int)f_r;
	float coef = (how_clear == 0) ? c->coe_clear   : c->coe_blocked;
	float spd  = (how_clear == 0) ? c->spd_clear   : c->spd_blocked;

	int steer = (int)((float)diff * coef);
	car_write_steer(steer);
	car_write_speed_ms(spd);
	car_pid_control();

	if (++run_div >= 5) {   /* 5 × 40 ms = 200 ms telemetry cadence */
		run_div = 0;
		send_telem(s, steer, spd);
		wifi_cmd_printf("$RUN:%d,%d\n", how_clear, steer);
	}
}

static void control_thread(void *a, void *b, void *c_)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c_);

	struct car_settings c;
	settings_get_copy(&c);
	int64_t next = k_uptime_get();

	LOG_INF("Control thread started");

	while (1) {
		wdt_feed_kick();

		int64_t now = k_uptime_get();
		if (now < next) {
			k_msleep((int32_t)(next - now));
			now = k_uptime_get();
		}
		settings_get_copy(&c);
		next = (next + c.loop_ms > now) ? next + c.loop_ms : now + c.loop_ms;

		bool drv_active = drv_enabled && manual_active &&
				  (k_uptime_get() - last_drv_ms < 500);

		if (drv_active) {
			int *s = sensors_poll();
			car_write_steer(manual_steer);
			car_write_speed_ms(manual_speed);
			car_pid_control();
			if (++run_div >= 5) {
				run_div = 0;
				send_telem(s, manual_steer, manual_speed);
			}
		} else if (running) {
			manual_active = false;
			work(&c);
		} else {
			int *s = sensors_poll();
			if (++run_div >= 5) {
				run_div = 0;
				send_telem(s, 0, 0.0f);
			}
		}
	}
}

void control_init(void)
{
	k_thread_create(&control_td, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack),
			control_thread, NULL, NULL, NULL,
			CONTROL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_td, "control");
}

void control_cmd_start(void)
{
	car_pid_reset();
	running = true;
	wifi_cmd_send("$STS:RUN\n");
}

void control_cmd_stop(void)
{
	running = false;
	manual_active = false;
	drv_enabled = false;
	car_write_speed(0);
	car_write_steer(0);
	wifi_cmd_send("$STS:STOP\n");
}

bool control_is_running(void)
{
	return running;
}

void control_set_manual(int steer, float speed)
{
	manual_steer = steer;
	manual_speed = speed;
	manual_active = true;
	last_drv_ms = k_uptime_get();
}

void control_set_drv_enabled(bool enabled)
{
	drv_enabled = enabled;
	if (!enabled) {
		manual_active = false;
		manual_steer = 0;
		manual_speed = 0.0f;
	}
}
