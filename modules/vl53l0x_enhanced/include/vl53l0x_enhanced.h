#pragma once

/*
 * vl53l0x_enhanced.h — Public API for enhanced VL53L0X Zephyr driver
 *
 * Custom sensor attributes, measurement modes, and profiles
 * for use with sensor_attr_set() / sensor_attr_get().
 */

#include <zephyr/drivers/sensor.h>

/* Include stock VL53L0X channel definitions for compatibility */
#include <zephyr/drivers/sensor/vl53l0x.h>

/* ─── Custom sensor attributes ───────────────────────────────────────────── */

enum sensor_attribute_vl53l0x {
	/** Device measurement mode (val1 = enum vl53l0x_mode).
	 *  Setting to continuous starts measurement automatically. */
	SENSOR_ATTR_VL53L0X_MODE = SENSOR_ATTR_PRIV_START,

	/** Timing budget in microseconds (val1 = µs, min ~20000).
	 *  Lower = faster but less accurate. Default 33000. */
	SENSOR_ATTR_VL53L0X_TIMING_BUDGET,

	/** Inter-measurement period in ms (val1 = ms).
	 *  Only used in CONTINUOUS_TIMED mode. */
	SENSOR_ATTR_VL53L0X_INTER_MEASUREMENT_PERIOD,

	/** VCSEL pulse period for pre-range phase (val1 = period). */
	SENSOR_ATTR_VL53L0X_VCSEL_PRE_RANGE,

	/** VCSEL pulse period for final-range phase (val1 = period). */
	SENSOR_ATTR_VL53L0X_VCSEL_FINAL_RANGE,

	/** Signal rate final-range limit in FixPoint16.16 MCPS (val1 = value).
	 *  Example: 0.25 MCPS = (FixPoint1616_t)(0.25 * 65536) = 16384 */
	SENSOR_ATTR_VL53L0X_SIGNAL_RATE_LIMIT,

	/** Sigma final-range limit in FixPoint16.16 mm (val1 = value).
	 *  Example: 32 mm = (FixPoint1616_t)(32 * 65536) = 2097152 */
	SENSOR_ATTR_VL53L0X_SIGMA_LIMIT,

	/** Apply a preset measurement profile (val1 = enum vl53l0x_profile).
	 *  Sets timing budget, signal/sigma limits, and VCSEL periods. */
	SENSOR_ATTR_VL53L0X_PROFILE,

	/** Offset calibration in micrometers (val1 = µm, signed). */
	SENSOR_ATTR_VL53L0X_OFFSET_CAL,

	/** Crosstalk compensation rate in FixPoint16.16 MCPS (val1 = value).
	 *  Setting a non-zero value enables crosstalk compensation. */
	SENSOR_ATTR_VL53L0X_XTALK_RATE,
};

/* ─── Measurement modes ──────────────────────────────────────────────────── */

enum vl53l0x_mode {
	/** Single-shot: one measurement per sample_fetch (blocking, ~33 ms). */
	VL53L0X_MODE_SINGLE          = 0,

	/** Continuous back-to-back: non-stop measurements.
	 *  sample_fetch returns -EAGAIN if no new data ready. */
	VL53L0X_MODE_CONTINUOUS      = 1,

	/** Continuous timed: measurements at inter_measurement_period interval.
	 *  sample_fetch returns -EAGAIN if no new data ready. */
	VL53L0X_MODE_CONTINUOUS_TIMED = 3,
};

/* ─── Measurement profiles (presets) ─────────────────────────────────────── */

enum vl53l0x_profile {
	/** Default: 33 ms budget, signal 0.1 MCPS, sigma 60 mm.
	 *  Good balance of speed and accuracy. Range ~2 m. */
	VL53L0X_PROFILE_DEFAULT       = 0,

	/** High speed: 20 ms budget, signal 0.25 MCPS, sigma 32 mm.
	 *  Fastest updates, good for short range (<1.2 m). */
	VL53L0X_PROFILE_HIGH_SPEED    = 1,

	/** Long range: 33 ms budget, signal 0.1 MCPS, sigma 60 mm.
	 *  Maximizes detection distance (~2 m). */
	VL53L0X_PROFILE_LONG_RANGE    = 2,

	/** High accuracy: 200 ms budget, signal 0.25 MCPS, sigma 32 mm.
	 *  Best accuracy, very slow updates. */
	VL53L0X_PROFILE_HIGH_ACCURACY = 3,
};
