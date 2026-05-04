/*
 * settings.c — Persistent configuration via Zephyr NVS
 *
 * Stores 34 runtime-configurable parameters to flash.
 * NVS key 1 = CarSettings blob + checksum.
 */

#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

LOG_MODULE_REGISTER(settings, LOG_LEVEL_INF);

/* ─── NVS setup ───────────────────────────────────────────────────────────── */
#define NVS_PARTITION      storage_partition
#define NVS_PARTITION_ID   PARTITION_ID(NVS_PARTITION)

#define NVS_KEY_SETTINGS   1

#define SETTINGS_MAGIC     0x554D4252  /* "UMBR" */
#define SETTINGS_VERSION   14

static struct nvs_fs nvs;
static bool nvs_ready;
static K_MUTEX_DEFINE(nvs_mutex);

/* ─── NVS storage structure (packed for flash) ────────────────────────────── */
struct __attribute__((packed)) nvs_settings {
	uint32_t magic;
	uint8_t  version;
	int16_t  front_obstacle_dist;
	int16_t  side_open_dist;
	int16_t  all_close_dist;
	int16_t  close_front_dist;
	float    pid_kp;
	float    pid_ki;
	float    pid_kd;
	int16_t  min_speed;
	int16_t  max_speed;
	int16_t  min_bspeed;
	uint8_t  min_point;
	uint8_t  max_point;
	uint8_t  neutral_point;
	int16_t  encoder_holes;
	float    wheel_diam_m;
	int16_t  loop_ms;
	float    spd_clear;
	float    spd_blocked;
	float    spd_slew;
	float    kick_pct;
	int16_t  kick_ms;
	float    coe_clear;
	float    coe_blocked;
	float    wrong_dir_deg;
	uint8_t  race_cw;
	int16_t  stuck_thresh;
	int16_t  stall_thresh;
	uint8_t  imu_rotate;
	uint8_t  servo_reverse;
	uint8_t  use_six_sensors;
	uint8_t  calibrated;
	uint8_t  bat_enabled;
	float    bat_multiplier;
	float    bat_low;
	int16_t  tach_glitch_filter_us;
	int16_t  reverse_time_ms;
	int16_t  turn_time_ms;
	float    reverse_speed;
	uint32_t checksum;
};

/* ─── Global configuration ────────────────────────────────────────────────── */
struct car_settings cfg;
static K_MUTEX_DEFINE(cfg_mutex);

/* ─── Defaults ────────────────────────────────────────────────────────────── */
static void set_defaults(void)
{
	cfg.front_obstacle_dist = DEFAULT_FOD;
	cfg.side_open_dist      = DEFAULT_SOD;
	cfg.all_close_dist      = DEFAULT_ACD;
	cfg.close_front_dist    = DEFAULT_CFD;
	cfg.pid_kp   = 66.4f;
	cfg.pid_ki   = 0.0f;
	cfg.pid_kd   = 4.16f;
	cfg.min_speed   = 1550;
	cfg.max_speed   = 1600;
	cfg.min_bspeed  = 1460;
	cfg.min_point     = 60;
	cfg.max_point     = 120;
	cfg.neutral_point = 90;
	cfg.encoder_holes = 68;
	cfg.wheel_diam_m  = 0.060f;
	cfg.loop_ms       = 40;
	cfg.spd_clear     = 0.4f;
	cfg.spd_blocked   = 0.3f;
	cfg.spd_slew      = 0.30f;
	cfg.kick_pct      = 25.0f;
	cfg.kick_ms       = 200;
	cfg.coe_clear     = 0.28f;
	cfg.coe_blocked   = 0.65f;
	cfg.wrong_dir_deg = 120.0f;
	cfg.race_cw       = true;
	cfg.stuck_thresh  = 25;
	cfg.stall_thresh  = 50;
	cfg.imu_rotate        = true;
	cfg.servo_reverse     = false;
	cfg.use_six_sensors   = true;
	cfg.calibrated        = false;
	cfg.bat_enabled    = false;
	cfg.bat_multiplier = 4.85f;
	cfg.bat_low        = 6.0f;
	cfg.tach_glitch_filter_us = 35;
	cfg.reverse_time_ms = 500;
	cfg.turn_time_ms    = 300;
	cfg.reverse_speed   = 0.3f;
}

static void sanitize_values(struct car_settings *c)
{
	c->front_obstacle_dist = CLAMP(c->front_obstacle_dist, 10, MAX_SENSOR_RANGE);
	c->side_open_dist = CLAMP(c->side_open_dist, 10, MAX_SENSOR_RANGE);
	c->all_close_dist = CLAMP(c->all_close_dist, 10, MAX_SENSOR_RANGE);
	c->close_front_dist = CLAMP(c->close_front_dist, 10, MAX_SENSOR_RANGE);

	c->min_speed = CLAMP(c->min_speed, NEUTRAL_SPEED, 2000);
	c->max_speed = CLAMP(c->max_speed, NEUTRAL_SPEED, 2000);
	c->min_bspeed = CLAMP(c->min_bspeed, 1000, NEUTRAL_SPEED);
	if (c->max_speed < c->min_speed) {
		c->max_speed = c->min_speed;
	}

	c->min_point = CLAMP(c->min_point, 0, 180);
	c->max_point = CLAMP(c->max_point, 0, 180);
	c->neutral_point = CLAMP(c->neutral_point, 0, 180);

	c->encoder_holes = CLAMP(c->encoder_holes, 1, 2000);
	c->loop_ms = CLAMP(c->loop_ms, 10, 1000);
	c->kick_ms = CLAMP(c->kick_ms, 0, 5000);
	c->stuck_thresh = CLAMP(c->stuck_thresh, 0, 1000);
	c->stall_thresh = CLAMP(c->stall_thresh, 0, 1000);
	c->tach_glitch_filter_us = CLAMP(c->tach_glitch_filter_us, 1, 500);
	c->reverse_time_ms = CLAMP(c->reverse_time_ms, 0, 5000);
	c->turn_time_ms    = CLAMP(c->turn_time_ms, 0, 5000);
	c->reverse_speed   = CLAMP(c->reverse_speed, 0.0f, 2.0f);
	if (!isfinite(c->reverse_speed)) c->reverse_speed = 0.3f;

	c->pid_kp = CLAMP(c->pid_kp, 0.0f, 5000.0f);
	c->pid_ki = CLAMP(c->pid_ki, 0.0f, 10000.0f);
	c->pid_kd = CLAMP(c->pid_kd, 0.0f, 2000.0f);
	c->wheel_diam_m = CLAMP(c->wheel_diam_m, 0.01f, 1.0f);
	c->spd_clear = CLAMP(c->spd_clear, 0.0f, 5.0f);
	c->spd_blocked = CLAMP(c->spd_blocked, 0.0f, 5.0f);
	c->spd_slew = CLAMP(c->spd_slew, 0.0f, 20.0f);
	c->kick_pct = CLAMP(c->kick_pct, 0.0f, 80.0f);
	c->coe_clear = CLAMP(c->coe_clear, 0.0f, 5.0f);
	c->coe_blocked = CLAMP(c->coe_blocked, 0.0f, 5.0f);
	c->wrong_dir_deg = CLAMP(c->wrong_dir_deg, 1.0f, 360.0f);
	c->bat_multiplier = CLAMP(c->bat_multiplier, 0.1f, 20.0f);
	c->bat_low = CLAMP(c->bat_low, 0.1f, 20.0f);

	if (!isfinite(c->pid_kp)) c->pid_kp = 0.0f;
	if (!isfinite(c->pid_ki)) c->pid_ki = 0.0f;
	if (!isfinite(c->pid_kd)) c->pid_kd = 0.0f;
	if (!isfinite(c->wheel_diam_m)) c->wheel_diam_m = 0.06f;
	if (!isfinite(c->spd_clear)) c->spd_clear = 0.0f;
	if (!isfinite(c->spd_blocked)) c->spd_blocked = 0.0f;
	if (!isfinite(c->spd_slew)) c->spd_slew = 0.0f;
	if (!isfinite(c->kick_pct)) c->kick_pct = 0.0f;
	if (!isfinite(c->coe_clear)) c->coe_clear = 0.0f;
	if (!isfinite(c->coe_blocked)) c->coe_blocked = 0.0f;
	if (!isfinite(c->wrong_dir_deg)) c->wrong_dir_deg = 120.0f;
	if (!isfinite(c->bat_multiplier)) c->bat_multiplier = 4.85f;
	if (!isfinite(c->bat_low)) c->bat_low = 6.0f;
}

static void sanitize_cfg(void)
{
	sanitize_values(&cfg);
}

/* ─── Checksum ────────────────────────────────────────────────────────────── */
static uint32_t compute_checksum(const struct nvs_settings *s)
{
	const uint8_t *p = (const uint8_t *)s;
	size_t len = offsetof(struct nvs_settings, checksum);

	return crc32_ieee(p, len);
}

/* ─── Pack cfg → nvs_settings ─────────────────────────────────────────────── */
static void populate_nvs(struct nvs_settings *s)
{
	s->magic   = SETTINGS_MAGIC;
	s->version = SETTINGS_VERSION;
	s->front_obstacle_dist = (int16_t)cfg.front_obstacle_dist;
	s->side_open_dist      = (int16_t)cfg.side_open_dist;
	s->all_close_dist      = (int16_t)cfg.all_close_dist;
	s->close_front_dist    = (int16_t)cfg.close_front_dist;
	s->pid_kp       = cfg.pid_kp;
	s->pid_ki       = cfg.pid_ki;
	s->pid_kd       = cfg.pid_kd;
	s->min_speed    = (int16_t)cfg.min_speed;
	s->max_speed    = (int16_t)cfg.max_speed;
	s->min_bspeed   = (int16_t)cfg.min_bspeed;
	s->min_point    = (uint8_t)cfg.min_point;
	s->max_point    = (uint8_t)cfg.max_point;
	s->neutral_point = (uint8_t)cfg.neutral_point;
	s->encoder_holes = (int16_t)cfg.encoder_holes;
	s->wheel_diam_m  = cfg.wheel_diam_m;
	s->loop_ms       = (int16_t)cfg.loop_ms;
	s->spd_clear     = cfg.spd_clear;
	s->spd_blocked   = cfg.spd_blocked;
	s->spd_slew      = cfg.spd_slew;
	s->kick_pct      = cfg.kick_pct;
	s->kick_ms       = (int16_t)cfg.kick_ms;
	s->coe_clear     = cfg.coe_clear;
	s->coe_blocked   = cfg.coe_blocked;
	s->wrong_dir_deg = cfg.wrong_dir_deg;
	s->race_cw       = cfg.race_cw ? 1 : 0;
	s->stuck_thresh  = (int16_t)cfg.stuck_thresh;
	s->stall_thresh  = (int16_t)cfg.stall_thresh;
	s->imu_rotate    = cfg.imu_rotate ? 1 : 0;
	s->servo_reverse    = cfg.servo_reverse ? 1 : 0;
	s->use_six_sensors  = cfg.use_six_sensors ? 1 : 0;
	s->calibrated       = cfg.calibrated ? 1 : 0;
	s->bat_enabled   = cfg.bat_enabled ? 1 : 0;
	s->bat_multiplier = cfg.bat_multiplier;
	s->bat_low       = cfg.bat_low;
	s->tach_glitch_filter_us = (int16_t)cfg.tach_glitch_filter_us;
	s->reverse_time_ms = (int16_t)cfg.reverse_time_ms;
	s->turn_time_ms    = (int16_t)cfg.turn_time_ms;
	s->reverse_speed   = cfg.reverse_speed;
	s->checksum      = compute_checksum(s);
}

/* ─── Unpack nvs_settings → cfg ───────────────────────────────────────────── */
static void apply_nvs(const struct nvs_settings *s)
{
	cfg.front_obstacle_dist = s->front_obstacle_dist;
	cfg.side_open_dist      = s->side_open_dist;
	cfg.all_close_dist      = s->all_close_dist;
	cfg.close_front_dist    = s->close_front_dist;
	cfg.pid_kp       = s->pid_kp;
	cfg.pid_ki       = s->pid_ki;
	cfg.pid_kd       = s->pid_kd;
	cfg.min_speed    = s->min_speed;
	cfg.max_speed    = s->max_speed;
	cfg.min_bspeed   = s->min_bspeed;
	cfg.min_point    = s->min_point;
	cfg.max_point    = s->max_point;
	cfg.neutral_point = s->neutral_point;
	cfg.encoder_holes = s->encoder_holes;
	cfg.wheel_diam_m  = s->wheel_diam_m;
	cfg.loop_ms       = s->loop_ms;
	cfg.spd_clear     = s->spd_clear;
	cfg.spd_blocked   = s->spd_blocked;
	cfg.spd_slew      = s->spd_slew;
	cfg.kick_pct      = s->kick_pct;
	cfg.kick_ms       = s->kick_ms;
	cfg.coe_clear     = s->coe_clear;
	cfg.coe_blocked   = s->coe_blocked;
	cfg.wrong_dir_deg = s->wrong_dir_deg;
	cfg.race_cw       = s->race_cw != 0;
	cfg.stuck_thresh  = s->stuck_thresh;
	cfg.stall_thresh  = s->stall_thresh;
	cfg.imu_rotate    = s->imu_rotate != 0;
	cfg.servo_reverse   = s->servo_reverse != 0;
	cfg.use_six_sensors = s->use_six_sensors != 0;
	cfg.calibrated      = s->calibrated != 0;
	cfg.bat_enabled   = s->bat_enabled != 0;
	cfg.bat_multiplier = s->bat_multiplier;
	cfg.bat_low       = s->bat_low;
	cfg.tach_glitch_filter_us = s->tach_glitch_filter_us;
	cfg.reverse_time_ms = s->reverse_time_ms;
	cfg.turn_time_ms    = s->turn_time_ms;
	cfg.reverse_speed   = s->reverse_speed;
	sanitize_cfg();
}

/* ─── Public API ──────────────────────────────────────────────────────────── */

void settings_init(void)
{
	k_mutex_lock(&cfg_mutex, K_FOREVER);
	set_defaults();
	sanitize_cfg();
	k_mutex_unlock(&cfg_mutex);

	const struct flash_area *fa;
	int rc = flash_area_open(NVS_PARTITION_ID, &fa);
	if (rc) {
		LOG_ERR("Flash area open failed: %d", rc);
		return;
	}

	nvs.flash_device = fa->fa_dev;
	nvs.offset = fa->fa_off;
	nvs.sector_size = 4096;
	nvs.sector_count = fa->fa_size / nvs.sector_size;
	flash_area_close(fa);

	rc = nvs_mount(&nvs);
	if (rc) {
		LOG_ERR("NVS mount failed: %d", rc);
		return;
	}

	nvs_ready = true;
	LOG_INF("NVS ready (%u sectors)", nvs.sector_count);
}

bool settings_load(void)
{
	if (!nvs_ready) {
		return false;
	}

	struct nvs_settings s;
	k_mutex_lock(&nvs_mutex, K_FOREVER);
	ssize_t len = nvs_read(&nvs, NVS_KEY_SETTINGS, &s, sizeof(s));
	k_mutex_unlock(&nvs_mutex);
	if (len != sizeof(s)) {
		LOG_INF("No saved settings (len=%zd)", len);
		return false;
	}

	if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) {
		LOG_WRN("Settings magic/version mismatch");
		return false;
	}

	if (compute_checksum(&s) != s.checksum) {
		LOG_WRN("Settings checksum mismatch");
		return false;
	}

	k_mutex_lock(&cfg_mutex, K_FOREVER);
	apply_nvs(&s);
	k_mutex_unlock(&cfg_mutex);
	LOG_INF("Settings loaded from NVS");
	return true;
}

bool settings_save(void)
{
	if (!nvs_ready) {
		return false;
	}

	struct nvs_settings s;
	k_mutex_lock(&cfg_mutex, K_FOREVER);
	sanitize_cfg();
	populate_nvs(&s);
	k_mutex_unlock(&cfg_mutex);

	k_mutex_lock(&nvs_mutex, K_FOREVER);
	ssize_t len = nvs_write(&nvs, NVS_KEY_SETTINGS, &s, sizeof(s));
	k_mutex_unlock(&nvs_mutex);
	if (len < 0) {
		LOG_ERR("NVS write failed: %zd", len);
		return false;
	}
	if (len != 0 && len != sizeof(s)) {
		LOG_ERR("NVS short write: %zd/%zu", len, sizeof(s));
		return false;
	}

	struct nvs_settings verify;
	k_mutex_lock(&nvs_mutex, K_FOREVER);
	ssize_t rlen = nvs_read(&nvs, NVS_KEY_SETTINGS, &verify, sizeof(verify));
	k_mutex_unlock(&nvs_mutex);
	if (rlen != sizeof(verify) ||
	    verify.magic != SETTINGS_MAGIC ||
	    verify.version != SETTINGS_VERSION ||
	    compute_checksum(&verify) != verify.checksum ||
	    memcmp(&verify, &s, sizeof(s)) != 0) {
		LOG_ERR("NVS write verify failed: %zd", rlen);
		return false;
	}

	LOG_INF("Settings saved to NVS (%zd bytes)", len);
	return true;
}

void settings_sanitize(struct car_settings *in_out)
{
	if (!in_out) {
		return;
	}
	sanitize_values(in_out);
}

void settings_set_copy(const struct car_settings *in)
{
	if (!in) {
		return;
	}

	struct car_settings next = *in;
	sanitize_values(&next);

	k_mutex_lock(&cfg_mutex, K_FOREVER);
	cfg = next;
	k_mutex_unlock(&cfg_mutex);
}

void settings_reset(void)
{
	k_mutex_lock(&cfg_mutex, K_FOREVER);
	set_defaults();
	sanitize_cfg();
	k_mutex_unlock(&cfg_mutex);
	LOG_INF("Settings reset to defaults");
}

void settings_lock(void)
{
	k_mutex_lock(&cfg_mutex, K_FOREVER);
}

void settings_unlock(void)
{
	k_mutex_unlock(&cfg_mutex);
}

void settings_get_copy(struct car_settings *out)
{
	if (!out) {
		return;
	}
	k_mutex_lock(&cfg_mutex, K_FOREVER);
	*out = cfg;
	k_mutex_unlock(&cfg_mutex);
}
