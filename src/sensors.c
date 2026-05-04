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

#include <errno.h>
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
#define VL53_ERROR_LIMIT 3
#define VL53_STALE_MS    300
#define VL53_STATUS_SIGNAL_FAIL 2
#define VL53_STATUS_NO_UPDATE   255

static int distances[SENSOR_COUNT]; /* cm×10 */
static int online_count;
static uint8_t error_count[SENSOR_COUNT];
static int64_t last_ok_ms[SENSOR_COUNT];
static bool fresh[SENSOR_COUNT];
static K_MUTEX_DEFINE(sensor_mutex);

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
		error_count[i] = 0;
		last_ok_ms[i] = 0;
		fresh[i] = false;
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
			vl53_valid[i] = false;
			online_count--;
		} else {
			cont_count++;
		}
	}

	LOG_INF("VL53L0X: %d/%d continuous mode", cont_count, online_count);
#else
	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		distances[i] = 9999;
		error_count[i] = 0;
		last_ok_ms[i] = 0;
		fresh[i] = false;
	}
	LOG_WRN("VL53 disabled by devicetree overlay (HIL no-sensors mode)");
#endif
}

/* ─── Poll ────────────────────────────────────────────────────────────────── */

#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
static void mark_invalid(int i)
{
	k_mutex_lock(&sensor_mutex, K_FOREVER);
	distances[i] = 9999;
	fresh[i] = false;
	k_mutex_unlock(&sensor_mutex);
}

static void mark_open(int i)
{
	k_mutex_lock(&sensor_mutex, K_FOREVER);
	distances[i] = 9999;
	error_count[i] = 0;
	last_ok_ms[i] = k_uptime_get();
	fresh[i] = true;
	k_mutex_unlock(&sensor_mutex);
}

static void mark_stale_if_needed(int i)
{
	k_mutex_lock(&sensor_mutex, K_FOREVER);
	if (last_ok_ms[i] > 0 &&
	    k_uptime_get() - last_ok_ms[i] > VL53_STALE_MS) {
		distances[i] = 9999;
		fresh[i] = false;
	}
	k_mutex_unlock(&sensor_mutex);
}

static void mark_error(int i)
{
	k_mutex_lock(&sensor_mutex, K_FOREVER);
	if (error_count[i] < UINT8_MAX) {
		error_count[i]++;
	}

	if (error_count[i] >= VL53_ERROR_LIMIT ||
	    (last_ok_ms[i] > 0 &&
	     k_uptime_get() - last_ok_ms[i] > VL53_STALE_MS)) {
		distances[i] = 9999;
		fresh[i] = false;
	}
	k_mutex_unlock(&sensor_mutex);
}

static void store_mm(int i, int mm)
{
	if (mm >= VL53L0X_MAX_RAW) {
		mark_open(i);
	} else if (mm <= 0) {
		mark_error(i);
	} else {
		k_mutex_lock(&sensor_mutex, K_FOREVER);
		distances[i] = (mm < MAX_SENSOR_RANGE) ? mm : MAX_SENSOR_RANGE;
		error_count[i] = 0;
		last_ok_ms[i] = k_uptime_get();
		fresh[i] = true;
		k_mutex_unlock(&sensor_mutex);
	}
}

static void poll_one(int i)
{
	if (!vl53_valid[i]) {
		mark_invalid(i);
		return;
	}

	int rc = sensor_sample_fetch(vl53_devs[i]);
	if (rc != 0) {
		if (rc == -EAGAIN) {
			mark_stale_if_needed(i);
			return;
		}
		mark_error(i);
		return;
	}

	struct sensor_value status;
	rc = sensor_channel_get(vl53_devs[i],
				(enum sensor_channel)SENSOR_CHAN_VL53L0X_RANGE_STATUS,
				&status);
	bool status_valid = (rc == 0);

	struct sensor_value val;
	rc = sensor_channel_get(vl53_devs[i], SENSOR_CHAN_DISTANCE, &val);
	if (rc != 0) {
		mark_error(i);
		return;
	}

	/* Convert meters.microns to mm (== cm×10) */
	int mm = val.val1 * 1000 + val.val2 / 1000;
	if (status_valid && status.val1 != 0) {
		if (status.val1 == VL53_STATUS_SIGNAL_FAIL &&
		    mm >= MAX_SENSOR_RANGE) {
			mark_open(i);
		} else if (status.val1 == VL53_STATUS_NO_UPDATE) {
			mark_stale_if_needed(i);
		} else {
			mark_error(i);
		}
		return;
	}
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

bool sensors_required_ready(bool use_six_sensors)
{
	uint8_t mask = BIT(IDX_FRONT_RIGHT) | BIT(IDX_RIGHT) |
		       BIT(IDX_LEFT) | BIT(IDX_FRONT_LEFT);
	bool ready = true;

	if (use_six_sensors) {
		mask |= BIT(IDX_HARD_RIGHT) | BIT(IDX_HARD_LEFT);
	}

	k_mutex_lock(&sensor_mutex, K_FOREVER);
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if ((mask & BIT(i)) == 0) {
			continue;
		}
		if (!fresh[i] || last_ok_ms[i] == 0 ||
		    k_uptime_get() - last_ok_ms[i] > VL53_STALE_MS) {
			ready = false;
			break;
		}
	}
	k_mutex_unlock(&sensor_mutex);

	return ready;
}

const int *sensors_get_distances(void)
{
	return distances;
}
