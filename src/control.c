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
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

#define CONTROL_STACK_SIZE 4096
#define CONTROL_PRIORITY   2

static K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_td;

struct control_state {
	bool running;
	bool drv_enabled;
	bool manual_active;   /* set by $DRV; expires after 500 ms */
	bool raw_esc_active;
	bool raw_servo_active;
	int manual_steer;
	float manual_speed;
	int raw_esc_us;
	int raw_servo_angle;
	int64_t last_drv_ms;
};

static struct control_state ctl;
static K_MUTEX_DEFINE(ctl_mutex);

extern void wdt_feed_kick(void);

static int run_div;
static uint32_t telem_count;

static void neutral_actuators(void)
{
	car_write_speed(0);
	car_write_steer(0);
	car_pid_reset();
}

static void clear_drive_command_locked(void)
{
	ctl.manual_active = false;
	ctl.raw_esc_active = false;
	ctl.raw_servo_active = false;
	ctl.manual_steer = 0;
	ctl.manual_speed = 0.0f;
	ctl.raw_esc_us = NEUTRAL_SPEED;
	ctl.raw_servo_angle = 90;
}

static void control_safe_stop(bool stop_running, bool disable_drive)
{
	k_mutex_lock(&ctl_mutex, K_FOREVER);
	if (stop_running) {
		ctl.running = false;
	}
	if (disable_drive) {
		ctl.drv_enabled = false;
	}
	clear_drive_command_locked();
	k_mutex_unlock(&ctl_mutex);

	neutral_actuators();
}

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
	gpio_pin_toggle_dt(&led);
	int *s = sensors_poll();
	if (!sensors_required_ready(c->use_six_sensors)) {
		control_safe_stop(true, true);
		wifi_cmd_send("$WARN:SENSOR_FAULT\n");
		wifi_cmd_send("$STS:STOP\n");
		return;
	}

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

		struct control_state snap;
		k_mutex_lock(&ctl_mutex, K_FOREVER);
		snap = ctl;
		k_mutex_unlock(&ctl_mutex);

		bool has_drive_cmd = snap.manual_active || snap.raw_esc_active ||
				     snap.raw_servo_active;
		bool drv_fresh = has_drive_cmd &&
				 (k_uptime_get() - snap.last_drv_ms < 500);
		bool drv_active = snap.drv_enabled && drv_fresh;

		if (drv_active) {
			int *s = sensors_poll();
			if (!sensors_required_ready(c.use_six_sensors)) {
				control_safe_stop(true, true);
				wifi_cmd_send("$WARN:SENSOR_FAULT\n");
				wifi_cmd_send("$STS:STOP\n");
				continue;
			}
			if (snap.raw_servo_active) {
				car_write_servo_raw(snap.raw_servo_angle);
			} else {
				car_write_steer(snap.manual_steer);
			}
			if (snap.raw_esc_active) {
				car_write_esc_us(snap.raw_esc_us);
			} else {
				car_write_speed_ms(snap.manual_speed);
				car_pid_control();
			}
			if (++run_div >= 5) {
				run_div = 0;
				send_telem(s, snap.manual_steer, snap.manual_speed);
			}
		} else if (snap.drv_enabled && has_drive_cmd && !drv_fresh) {
			control_safe_stop(false, false);
		} else if (snap.running) {
			k_mutex_lock(&ctl_mutex, K_FOREVER);
			clear_drive_command_locked();
			k_mutex_unlock(&ctl_mutex);
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
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	k_thread_create(&control_td, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack),
			control_thread, NULL, NULL, NULL,
			CONTROL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_td, "control");
}

void control_cmd_start(void)
{
	struct car_settings c;
	settings_get_copy(&c);
	if (!sensors_required_ready(c.use_six_sensors)) {
		wifi_cmd_send("$NAK:sensor_fault\n");
		return;
	}

	car_pid_reset();
	k_mutex_lock(&ctl_mutex, K_FOREVER);
	ctl.running = true;
	clear_drive_command_locked();
	k_mutex_unlock(&ctl_mutex);
	wifi_cmd_send("$STS:RUN\n");
}

void control_cmd_stop(void)
{
	control_safe_stop(true, true);
	wifi_cmd_send("$STS:STOP\n");
}

bool control_is_running(void)
{
	bool is_running;
	k_mutex_lock(&ctl_mutex, K_FOREVER);
	is_running = ctl.running;
	k_mutex_unlock(&ctl_mutex);
	return is_running;
}

bool control_drive_enabled(void)
{
	bool enabled;
	k_mutex_lock(&ctl_mutex, K_FOREVER);
	enabled = ctl.drv_enabled;
	k_mutex_unlock(&ctl_mutex);
	return enabled;
}

bool control_set_manual(int steer, float speed)
{
	bool accepted = false;

	k_mutex_lock(&ctl_mutex, K_FOREVER);
	if (ctl.drv_enabled) {
		ctl.manual_steer = steer;
		ctl.manual_speed = speed;
		ctl.manual_active = true;
		ctl.raw_esc_active = false;
		ctl.raw_servo_active = false;
		ctl.last_drv_ms = k_uptime_get();
		accepted = true;
	}
	k_mutex_unlock(&ctl_mutex);

	return accepted;
}

bool control_set_raw_esc_us(int us)
{
	bool accepted = false;

	k_mutex_lock(&ctl_mutex, K_FOREVER);
	if (ctl.drv_enabled) {
		ctl.raw_esc_us = CLAMP(us, 1000, 2000);
		ctl.raw_esc_active = true;
		ctl.raw_servo_active = false;
		ctl.raw_servo_angle = 90;
		ctl.manual_active = false;
		ctl.manual_speed = 0.0f;
		ctl.manual_steer = 0;
		ctl.last_drv_ms = k_uptime_get();
		accepted = true;
	}
	k_mutex_unlock(&ctl_mutex);

	return accepted;
}

bool control_set_raw_servo(int angle)
{
	bool accepted = false;

	k_mutex_lock(&ctl_mutex, K_FOREVER);
	if (ctl.drv_enabled) {
		ctl.raw_servo_angle = CLAMP(angle, 0, 180);
		ctl.raw_servo_active = true;
		ctl.raw_esc_active = false;
		ctl.raw_esc_us = NEUTRAL_SPEED;
		ctl.manual_active = false;
		ctl.manual_steer = 0;
		ctl.manual_speed = 0.0f;
		ctl.last_drv_ms = k_uptime_get();
		accepted = true;
	}
	k_mutex_unlock(&ctl_mutex);

	return accepted;
}

void control_set_drv_enabled(bool enabled)
{
	k_mutex_lock(&ctl_mutex, K_FOREVER);
	ctl.drv_enabled = enabled;
	if (!enabled) {
		clear_drive_command_locked();
	}
	k_mutex_unlock(&ctl_mutex);

	if (!enabled) {
		neutral_actuators();
	}
}
