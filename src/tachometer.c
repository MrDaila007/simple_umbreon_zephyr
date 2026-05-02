/*
 * tachometer.c — Optical encoder speed measurement
 *
 * Ported from luna_car.h: taho_interrupt(), get_speed()
 * Uses GPIO interrupt on GP13 (RISING edge) with atomic variables.
 */

#include "tachometer.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(tachometer, LOG_LEVEL_INF);

/* ─── GPIO ────────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec tach_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(tach_pin), gpios);
static struct gpio_callback tach_cb_data;

/* ─── Shared state (ISR writes, main reads — all atomic) ──────────────────── */
static atomic_t taho_count     = ATOMIC_INIT(0);
static atomic_t taho_last_cyc  = ATOMIC_INIT(0);  /* raw HW cycles */
static atomic_t taho_interval  = ATOMIC_INIT(0);   /* delta in µs */
static atomic_t taho_glitch_filter_us = ATOMIC_INIT(35);

/* ─── Cycle-domain helpers ────────────────────────────────────────────────── */
/* Subtract in cycle domain (uint32_t wrap is correct), then convert to µs.
 * k_cycle_get_32() is ISR-safe (reads HW register) and has full clock
 * resolution (~6.7 ns at 150 MHz).  Wraparound every ~28 s is handled
 * by unsigned subtraction on the raw cycle values. */
static inline uint32_t cyc_delta_us(uint32_t from_cyc, uint32_t to_cyc)
{
	uint32_t delta_cyc = to_cyc - from_cyc;          /* handles wrap */
	return (uint32_t)k_cyc_to_us_floor64(delta_cyc); /* small value — safe */
}

/* Reject only narrow EMI glitches on the optical line — not valid slot edges.
 * 200 µs here falsely dropped real pulses (~15+ m/s with 68 holes on Ø65 mm ≈ 200 µs). */
static inline uint32_t clamp_glitch_filter_us(uint32_t us)
{
	if (us < 1U) {
		return 1U;
	}
	if (us > 500U) {
		return 500U;
	}
	return us;
}

/* ─── ISR ─────────────────────────────────────────────────────────────────── */
/* Port of taho_interrupt() from luna_car.h:71-78 */
static void tach_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	uint32_t now_cyc = k_cycle_get_32();
	uint32_t last_cyc = (uint32_t)atomic_get(&taho_last_cyc);
	uint32_t delta_us = cyc_delta_us(last_cyc, now_cyc);

	uint32_t glitch_filter_us = (uint32_t)atomic_get(&taho_glitch_filter_us);
	if (delta_us < glitch_filter_us) {
		return;
	}

	atomic_inc(&taho_count);
	atomic_set(&taho_interval, (atomic_val_t)delta_us);
	atomic_set(&taho_last_cyc, (atomic_val_t)now_cyc);
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void taho_init(void)
{
	struct car_settings c;
	settings_get_copy(&c);
	atomic_set(&taho_glitch_filter_us, (atomic_val_t)clamp_glitch_filter_us(
		(uint32_t)c.tach_glitch_filter_us));

	if (!gpio_is_ready_dt(&tach_gpio)) {
		LOG_ERR("Tachometer GPIO not ready");
		return;
	}

	gpio_pin_configure_dt(&tach_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&tach_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&tach_cb_data, tach_isr, BIT(tach_gpio.pin));
	gpio_add_callback(tach_gpio.port, &tach_cb_data);

	LOG_INF("Tachometer init (GP13, RISING edge)");
}

/* ─── Speed calculation ───────────────────────────────────────────────────── */
/* Port of get_speed() from luna_car.h:81-87 */

float taho_get_speed(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	uint32_t now_cyc = k_cycle_get_32();
	uint32_t last_cyc = (uint32_t)atomic_get(&taho_last_cyc);
	uint32_t iv = (uint32_t)atomic_get(&taho_interval);

	uint32_t elapsed = cyc_delta_us(last_cyc, now_cyc);
	if (elapsed < iv) {
		elapsed = iv;
	}

	if (iv == 0 || elapsed > 500000 || c.encoder_holes <= 0) {
		return 0.0f;
	}

	return ((float)M_PI * c.wheel_diam_m) /
	       ((float)c.encoder_holes * ((float)elapsed / 1e6f));
}

uint32_t taho_get_count(void)
{
	return (uint32_t)atomic_get(&taho_count);
}

uint32_t taho_time_since_last_us(void)
{
	uint32_t now_cyc = k_cycle_get_32();
	uint32_t last_cyc = (uint32_t)atomic_get(&taho_last_cyc);
	return cyc_delta_us(last_cyc, now_cyc);
}

void taho_reset(void)
{
	atomic_set(&taho_count, 0);
	atomic_set(&taho_last_cyc, (atomic_val_t)k_cycle_get_32());
	atomic_set(&taho_interval, 0);
}

void taho_set_glitch_filter_us(uint32_t us)
{
	atomic_set(&taho_glitch_filter_us, (atomic_val_t)clamp_glitch_filter_us(us));
}
