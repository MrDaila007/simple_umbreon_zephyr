#pragma once

#include <stdbool.h>

/* ─── Car control: PWM servo + ESC ────────────────────────────────────────── */

void car_init(void);

/* Steering: -1000 (left) … +1000 (right) */
void car_write_steer(int s);

/* Direct servo angle write (0–180°) for calibration */
void car_write_servo_raw(int angle);

/* Direct ESC µs write (1000–2000) for calibration */
void car_write_esc_us(int us);

/* Raw speed: -1000 … +1000, bypasses PID */
void car_write_speed(int s);

/* Set target speed for PID (m/s) */
void car_write_speed_ms(float target);

/* Run PID control tick — call from control loop */
void car_pid_control(void);

/* PID state reset */
void car_pid_reset(void);

/* ESC calibration: max → min → neutral */
void car_run_calibration(void);
