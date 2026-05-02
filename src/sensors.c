/*
 * sensors.c — 6× VL53L0X ToF sensor array
 *
 * Uses the enhanced VL53L0X driver (out-of-tree module) which supports
 * continuous back-to-back measurement mode via standard Zephyr sensor API.
 * Non-blocking reads (~1 ms/sensor instead of ~33 ms in single-shot).
 *
 * Sensor order: [Hard-Right, Front-Right, Right, Left, Front-Left, Hard-Left]
 */

#include "sensors.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
#include <vl53l0x_enhanced.h>
#endif

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

#define VL53L0X_MAX_RAW  8190  /* sensor overflow / out-of-range indicator */

static int distances[SENSOR_COUNT]; /* cm×10 */
static int online_count;

#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
/* ─── Device handles ──────────────────────────────────────────────────────── */
static const struct device *vl53_devs[SENSOR_COUNT];
static bool vl53_valid[SENSOR_COUNT];

extern void wdt_feed_kick(void);

/* ─── Sensor nodelabel → device mapping ───────────────────────────────────── */
#define VL53_DEV(idx, label) \
	vl53_devs[idx] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(label))
#endif

/* ─── Init ────────────────────────────────────────────────────────────────── */

void sensors_init(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	/* Try I2C bus recovery before initializing sensors */
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (device_is_ready(i2c1)) {
		int rc = i2c_recover_bus(i2c1);
		if (rc == 0) {
			LOG_INF("I2C1 bus recovery OK");
		} else if (rc == -ENOSYS) {
			LOG_DBG("I2C1 bus recovery not supported");
		} else {
			LOG_WRN("I2C1 bus recovery failed: %d", rc);
		}
	}

	/* Get device handles — order matches sensor_config.h indices */
	VL53_DEV(0, vl53l0x_0);  /* IDX_HARD_RIGHT  — XSHUT GP6  */
	VL53_DEV(1, vl53l0x_1);  /* IDX_FRONT_RIGHT — XSHUT GP7  */
	VL53_DEV(2, vl53l0x_2);  /* IDX_RIGHT       — XSHUT GP8  */
	VL53_DEV(3, vl53l0x_3);  /* IDX_LEFT        — XSHUT GP9  */
	VL53_DEV(4, vl53l0x_4);  /* IDX_FRONT_LEFT  — XSHUT GP14 */
	VL53_DEV(5, vl53l0x_5);  /* IDX_HARD_LEFT   — XSHUT GP15 */

	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		distances[i] = 9999;
		if (vl53_devs[i] && device_is_ready(vl53_devs[i])) {
			vl53_valid[i] = true;
			online_count++;
		} else {
			vl53_valid[i] = false;
			LOG_WRN("VL53L0X[%d] not ready", i);
		}
	}

	LOG_INF("VL53L0X: %d/%d online", online_count, SENSOR_COUNT);

	/*
	 * Phase 2: trigger lazy init (calibration) via first blocking fetch,
	 * then configure high-speed profile and continuous mode.
	 */
	int cont_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (!vl53_valid[i]) {
			continue;
		}

		/* First fetch triggers lazy init: XSHUT release, address reconfig,
		 * DataInit, StaticInit, calibration (~50 ms per sensor). */
		int rc = sensor_sample_fetch(vl53_devs[i]);
		wdt_feed_kick();
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] init fetch failed: %d", i, rc);
			vl53_valid[i] = false;
			online_count--;
			continue;
		}

		/* Set high-speed profile (20 ms timing budget) */
		struct sensor_value val = { .val1 = VL53L0X_PROFILE_HIGH_SPEED };
		rc = sensor_attr_set(vl53_devs[i], SENSOR_CHAN_DISTANCE,
				     (enum sensor_attribute)SENSOR_ATTR_VL53L0X_PROFILE,
				     &val);
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] profile set failed: %d", i, rc);
		}

		/* Switch to continuous back-to-back mode */
		val.val1 = VL53L0X_MODE_CONTINUOUS;
		rc = sensor_attr_set(vl53_devs[i], SENSOR_CHAN_DISTANCE,
				     (enum sensor_attribute)SENSOR_ATTR_VL53L0X_MODE,
				     &val);
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] continuous start failed: %d", i, rc);
		} else {
			cont_count++;
		}
	}

	LOG_INF("VL53L0X: %d/%d continuous mode", cont_count, online_count);
#else
	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		distances[i] = 9999;
	}
	LOG_WRN("VL53 disabled by devicetree overlay (HIL no-sensors mode)");
#endif
}

/* ─── Poll ────────────────────────────────────────────────────────────────── */

#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
static void store_mm(int i, int mm)
{
	if (mm >= VL53L0X_MAX_RAW || mm <= 0) {
		distances[i] = 9999;
	} else {
		distances[i] = (mm < MAX_SENSOR_RANGE) ? mm : MAX_SENSOR_RANGE;
	}
}

static void poll_one(int i)
{
	if (!vl53_valid[i]) {
		distances[i] = 9999;
		return;
	}

	int rc = sensor_sample_fetch(vl53_devs[i]);
	if (rc != 0) {
		return; /* -EAGAIN (no new data) or error: keep previous value */
	}

	struct sensor_value val;
	rc = sensor_channel_get(vl53_devs[i], SENSOR_CHAN_DISTANCE, &val);
	if (rc != 0) {
		return;
	}

	/* Convert meters.microns to mm (== cm×10) */
	int mm = val.val1 * 1000 + val.val2 / 1000;
	store_mm(i, mm);
}
#endif

int *sensors_poll(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	for (int i = 0; i < SENSOR_COUNT; i++) {
		poll_one(i);
	}
#endif
	return distances;
}

int *sensors_poll_mask(uint8_t mask)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (mask & BIT(i)) {
			poll_one(i);
		}
	}
#else
	ARG_UNUSED(mask);
#endif
	return distances;
}

int sensors_online_count(void)
{
	return online_count;
}

const int *sensors_get_distances(void)
{
	return distances;
}
