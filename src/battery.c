/*
 * battery.c — ADC battery voltage monitor
 *
 * Reads GP28 (ADC channel 2) and scales by cfg.bat_multiplier to recover
 * the actual pack voltage. Thread-safe; called from both the control thread
 * (telemetry) and the wifi_cmd thread ($BAT command).
 */

#include "battery.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define BAT_ADC_NODE   DT_NODELABEL(adc)
#define BAT_CH         2    /* GP28 = ADC channel 2 on RP2350 */
#define BAT_RESOLUTION 12

static const struct device *adc_dev;
static bool                 adc_ready;
static K_MUTEX_DEFINE(bat_mutex);

static const struct adc_channel_cfg bat_ch_cfg = {
	.gain             = ADC_GAIN_1,
	.reference        = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id       = BAT_CH,
};

static float last_voltage;

void battery_init(void)
{
	adc_dev = DEVICE_DT_GET(BAT_ADC_NODE);
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC not ready — battery monitor disabled");
		return;
	}

	int rc = adc_channel_setup(adc_dev, &bat_ch_cfg);
	if (rc) {
		LOG_ERR("ADC channel %d setup failed: %d", BAT_CH, rc);
		return;
	}

	adc_ready = true;
	LOG_INF("Battery ADC ready (GP28, channel %d)", BAT_CH);
}

float battery_voltage(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	if (!adc_ready || !c.bat_enabled) {
		return 0.0f;
	}

	int16_t sample = 0;
	struct adc_sequence seq = {
		.channels    = BIT(BAT_CH),
		.buffer      = &sample,
		.buffer_size = sizeof(sample),
		.resolution  = BAT_RESOLUTION,
	};

	k_mutex_lock(&bat_mutex, K_FOREVER);
	int rc = adc_read(adc_dev, &seq);
	k_mutex_unlock(&bat_mutex);

	if (rc < 0) {
		LOG_DBG("ADC read error: %d", rc);
		return last_voltage;
	}

	int32_t mv = sample;
	adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1,
			      BAT_RESOLUTION, &mv);

	last_voltage = (mv / 1000.0f) * c.bat_multiplier;
	return last_voltage;
}

bool battery_is_low(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	if (!c.bat_enabled) {
		return false;
	}

	float v = battery_voltage();
	return (v > 0.1f) && (v < c.bat_low);
}
