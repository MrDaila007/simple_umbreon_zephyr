/*
 * car.c — PWM servo/ESC control + PID speed controller
 *
 * Ported from luna_car.h: write_steer(), write_speed(), pid_control_motor()
 */

#include "car.h"
#include "settings.h"
#include "tachometer.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(car, LOG_LEVEL_INF);

/* ─── PWM devices ─────────────────────────────────────────────────────────── */
static const struct pwm_dt_spec servo_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
static const struct pwm_dt_spec esc_pwm   = PWM_DT_SPEC_GET(DT_NODELABEL(esc));

/* ─── PID state ───────────────────────────────────────────────────────────── */
static float target_speed;
static float pid_ref_applied; /* setpoint after spd_slew limiting */
static float pid_integral;
static float pid_prev_filtered;  /* for derivative-on-measurement */
static float pid_filtered;
static uint32_t pid_prev_cnt;
static int64_t pid_prev_ms;
/* Start boost: edge on forward command from rest */
static int64_t kick_until_ms;
static float kick_prev_pid_ref;

/* ─── Helpers ─────────────────────────────────────────────────────────────── */

/* Convert microseconds to PWM pulse width (in nanoseconds for Zephyr API) */
static inline void servo_set_us(int us)
{
	pwm_set_pulse_dt(&servo_pwm, PWM_USEC(us));
}

static inline void esc_set_us(int us)
{
	pwm_set_pulse_dt(&esc_pwm, PWM_USEC(us));
}

/* Map a value from one range to another (Arduino map() equivalent) */
static inline int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void car_init(void)
{
	if (!pwm_is_ready_dt(&servo_pwm)) {
		LOG_ERR("Servo PWM not ready");
	}
	if (!pwm_is_ready_dt(&esc_pwm)) {
		LOG_ERR("ESC PWM not ready");
	}

	/* Neutral position */
	car_write_steer(0);
	car_write_speed(0);

	LOG_INF("Car PWM init done (servo GP10, ESC GP11)");
}

/* ─── Steering ────────────────────────────────────────────────────────────── */
/* Port of Car::write_steer() from luna_car.h:408-415 */

void car_write_steer(int s)
{
	struct car_settings c;
	settings_get_copy(&c);

	if (c.servo_reverse) {
		s = -s;
	}
	s = CLAMP(s, -1000, 1000);

	int angle;
	if (s < 0) {
		angle = map_val(s, -1000, 0, c.min_point, c.neutral_point);
	} else {
		angle = map_val(s, 0, 1000, c.neutral_point, c.max_point);
	}

	/* Convert angle (0–180°) to pulse width (700–2300 µs) */
	int pulse_us = 700 + angle * 1600 / 180;
	servo_set_us(pulse_us);
}

void car_write_servo_raw(int angle)
{
	angle = CLAMP(angle, 0, 180);
	int pulse_us = 700 + angle * 1600 / 180;
	servo_set_us(pulse_us);
}

/* ─── ESC ─────────────────────────────────────────────────────────────────── */

void car_write_esc_us(int us)
{
	us = CLAMP(us, 1000, 2000);
	esc_set_us(us);
}

/* Port of Car::write_speed() from luna_car.h:396-405 */
void car_write_speed(int s)
{
	struct car_settings c;
	settings_get_copy(&c);

	/* Reset PID state — direct control bypasses PID */
	car_pid_reset();

	s = CLAMP(s, -1000, 1000);
	int us;
	if (s > 0) {
		us = map_val(s, 1, 1000, c.min_speed, c.max_speed);
	} else if (s < 0) {
		us = map_val(s, -1000, -1, 1000, c.min_bspeed);
	} else {
		us = NEUTRAL_SPEED;
	}
	esc_set_us(us);
}

void car_write_speed_ms(float target)
{
	target_speed = target;
}

/* ─── PID ─────────────────────────────────────────────────────────────────── */
/* Port of Car::pid_control_motor() from luna_car.h:350-390 */

void car_pid_reset(void)
{
	pid_integral = 0;
	pid_prev_filtered = 0;
	pid_filtered = 0;
	pid_prev_cnt = taho_get_count();
	pid_prev_ms = 0;
	target_speed = 0;
	pid_ref_applied = 0;
	kick_until_ms = 0;
	kick_prev_pid_ref = 0;
}

void car_pid_control(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	int64_t now_ms = k_uptime_get();
	if (pid_prev_ms == 0) {
		pid_prev_ms = now_ms;
		return;
	}

	float dt = (now_ms - pid_prev_ms) / 1000.0f;
	if (dt < 0.01f) {
		return;
	}
	pid_prev_ms = now_ms;

	/* Count-based speed measurement */
	uint32_t cnt = taho_get_count();
	uint32_t delta_cnt = cnt - pid_prev_cnt;
	pid_prev_cnt = cnt;

	float raw_speed = 0.0f;
	if (c.encoder_holes > 0) {
		raw_speed = (delta_cnt / (float)c.encoder_holes) *
			    ((float)M_PI * c.wheel_diam_m) / dt;
	}

	/* EMA filter (0.7 = responsive, 0.3 = smooth) */
	pid_filtered = 0.7f * raw_speed + 0.3f * pid_filtered;
	if (taho_time_since_last_us() > 500000) {
		pid_filtered = 0;
	}

	/* Slew-limit setpoint (m/s²) — smooth start / mode transitions */
	if (c.spd_slew > 0.001f) {
		float step = c.spd_slew * dt;
		if (target_speed > pid_ref_applied + step) {
			pid_ref_applied += step;
		} else if (target_speed < pid_ref_applied - step) {
			pid_ref_applied -= step;
		} else {
			pid_ref_applied = target_speed;
		}
	} else {
		pid_ref_applied = target_speed;
	}

	/* Kick-off pulse: % of (max−min) ESC span for kick_ms after forward edge from rest */
	bool want_fwd = pid_ref_applied > 0.02f;
	bool had_fwd = kick_prev_pid_ref > 0.02f;
	if (!want_fwd) {
		kick_until_ms = 0;
	} else if (!had_fwd && pid_filtered < 0.10f && c.kick_pct > 0.05f &&
		   c.kick_ms > 0) {
		kick_until_ms = now_ms + c.kick_ms;
	}
	if (want_fwd && pid_ref_applied > 0.05f &&
	    pid_filtered >= pid_ref_applied * 0.78f) {
		kick_until_ms = 0;
	}
	kick_prev_pid_ref = pid_ref_applied;

	/* PID — derivative on measurement (no kick on target change) */
	float error = pid_ref_applied - pid_filtered;
	pid_integral += error * dt;
	pid_integral = CLAMP(pid_integral, -50.0f, 50.0f);
	float deriv = -(pid_filtered - pid_prev_filtered) / dt;
	pid_prev_filtered = pid_filtered;

	/* Feedforward: jump past motor dead zone (follows slewed setpoint) */
	float ff = (pid_ref_applied > 0.01f) ? (float)(c.min_speed - NEUTRAL_SPEED) : 0;
	float output = ff + c.pid_kp * error + c.pid_ki * pid_integral + c.pid_kd * deriv;

	float kick_us = 0.0f;
	if (c.kick_pct > 0.05f && want_fwd && now_ms < kick_until_ms) {
		int span = c.max_speed - c.min_speed;
		if (span < 1) {
			span = 1;
		}
		kick_us = (c.kick_pct / 100.0f) * (float)span;
		if (kick_us > 40.0f) {
			kick_us = 40.0f;
		}
	}

	int esc_val = NEUTRAL_SPEED + (int)(output + kick_us);
	esc_val = CLAMP(esc_val, NEUTRAL_SPEED, c.max_speed);
	esc_set_us(esc_val);
}

/* ─── ESC Calibration ─────────────────────────────────────────────────────── */
/* Port of run_calibration() from Umbreon_roborace.ino:1030-1057 */

void car_run_calibration(void)
{
	extern void wdt_feed_kick(void);
	extern void wifi_cmd_send(const char *str);

	LOG_INF("ESC calibration: max signal");
	wifi_cmd_send("$T:CAL,phase=esc_max\n");
	esc_set_us(2000);
	k_msleep(3000);
	wdt_feed_kick();

	LOG_INF("ESC calibration: min signal");
	wifi_cmd_send("$T:CAL,phase=esc_min\n");
	esc_set_us(1000);
	k_msleep(3000);
	wdt_feed_kick();

	LOG_INF("ESC calibration: neutral");
	wifi_cmd_send("$T:CAL,phase=esc_neutral\n");
	esc_set_us(NEUTRAL_SPEED);
	k_msleep(1000);
	wdt_feed_kick();

	settings_lock();
	cfg.calibrated = true;
	settings_unlock();
	settings_save();

	wifi_cmd_send("$T:CAL,phase=done\n");
	wifi_cmd_send("$TDONE:cal\n");
	LOG_INF("ESC calibration complete");
}
