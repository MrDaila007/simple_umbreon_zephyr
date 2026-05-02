/*
 * vl53l0x_enhanced.c — Enhanced VL53L0X Zephyr sensor driver
 *
 * Replaces the stock st,vl53l0x driver. Adds:
 *  - Continuous back-to-back and timed measurement modes
 *  - Configurable timing budget, VCSEL periods, signal/sigma limits
 *  - Measurement profiles (high-speed, long-range, high-accuracy)
 *  - Offset and crosstalk calibration via sensor_attr_set()
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l0x

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/vl53l0x.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_enhanced.h"

LOG_MODULE_REGISTER(VL53L0X_ENH, CONFIG_SENSOR_LOG_LEVEL);

/* ─── Constants (matching stock driver defaults) ─────────────────────────── */
#define VL53L0X_INITIAL_ADDR           0x29
#define VL53L0X_REG_WHO_AM_I           0xC0
#define VL53L0X_CHIP_ID                0xEEAA

/* Default profile values */
#define DEFAULT_TIMING_BUDGET          33000
#define DEFAULT_SIGNAL_LIMIT           ((FixPoint1616_t)(0.1 * 65536))
#define DEFAULT_SIGMA_LIMIT            ((FixPoint1616_t)(60 * 65536))
#define DEFAULT_VCSEL_PRE_RANGE        18
#define DEFAULT_VCSEL_FINAL_RANGE      14

/* tBOOT (1.2 ms max.) */
#define T_BOOT K_USEC(1200)

/* ─── Driver data structures ─────────────────────────────────────────────── */

struct vl53l0x_enh_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec xshut;
};

struct vl53l0x_enh_data {
	bool started;
	bool continuous_running;
	VL53L0X_Dev_t vl53l0x;
	VL53L0X_RangingMeasurementData_t ranging_data;
	VL53L0X_DeviceModes current_mode;
};

/* ─── Profile application ────────────────────────────────────────────────── */

static int apply_profile(struct vl53l0x_enh_data *data, enum vl53l0x_profile profile)
{
	VL53L0X_DEV dev = &data->vl53l0x;
	VL53L0X_Error ret;
	bool was_running = data->continuous_running;

	if (was_running) {
		VL53L0X_StopMeasurement(dev);
		data->continuous_running = false;
	}

	switch (profile) {
	case VL53L0X_PROFILE_DEFAULT:
	case VL53L0X_PROFILE_LONG_RANGE:
		ret  = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t)(0.1 * 65536));
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			(FixPoint1616_t)(60 * 65536));
		ret |= VL53L0X_SetVcselPulsePeriod(dev,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		ret |= VL53L0X_SetVcselPulsePeriod(dev,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
		break;

	case VL53L0X_PROFILE_HIGH_SPEED:
		ret  = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 20000);
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t)(0.25 * 65536));
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			(FixPoint1616_t)(32 * 65536));
		break;

	case VL53L0X_PROFILE_HIGH_ACCURACY:
		ret  = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 200000);
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t)(0.25 * 65536));
		ret |= VL53L0X_SetLimitCheckValue(dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			(FixPoint1616_t)(32 * 65536));
		break;

	default:
		return -EINVAL;
	}

	if (ret) {
		LOG_ERR("apply_profile failed: %d", ret);
		return -EIO;
	}

	if (was_running) {
		ret = VL53L0X_StartMeasurement(dev);
		if (ret) {
			return -EIO;
		}
		data->continuous_running = true;
	}

	return 0;
}

/* ─── Hardware init (lazy, called on first sample_fetch) ─────────────────── */

static int vl53l0x_enh_setup(const struct device *dev)
{
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_DEV vl = &data->vl53l0x;
	VL53L0X_Error ret;
	uint8_t vhv, phase_cal;
	uint32_t ref_spad_count;
	uint8_t is_aperture_spads;

	ret = VL53L0X_StaticInit(vl);
	if (ret) {
		LOG_ERR("[%s] StaticInit failed: %d", dev->name, ret);
		return -EIO;
	}

	ret = VL53L0X_PerformRefCalibration(vl, &vhv, &phase_cal);
	if (ret) {
		LOG_ERR("[%s] RefCalibration failed: %d", dev->name, ret);
		return -EIO;
	}

	ret = VL53L0X_PerformRefSpadManagement(vl, &ref_spad_count,
						&is_aperture_spads);
	if (ret) {
		LOG_ERR("[%s] RefSpadManagement failed: %d", dev->name, ret);
		return -EIO;
	}

	/* Default mode: single-shot */
	ret = VL53L0X_SetDeviceMode(vl, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		LOG_ERR("[%s] SetDeviceMode failed: %d", dev->name, ret);
		return -EIO;
	}
	data->current_mode = VL53L0X_DEVICEMODE_SINGLE_RANGING;

	/* Enable limit checks */
	ret  = VL53L0X_SetLimitCheckEnable(vl,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	ret |= VL53L0X_SetLimitCheckEnable(vl,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	if (ret) {
		LOG_ERR("[%s] SetLimitCheckEnable failed: %d", dev->name, ret);
		return -EIO;
	}

	/* Apply default profile */
	ret  = VL53L0X_SetLimitCheckValue(vl,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, DEFAULT_SIGNAL_LIMIT);
	ret |= VL53L0X_SetLimitCheckValue(vl,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, DEFAULT_SIGMA_LIMIT);
	ret |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl,
		DEFAULT_TIMING_BUDGET);
	ret |= VL53L0X_SetVcselPulsePeriod(vl,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, DEFAULT_VCSEL_PRE_RANGE);
	ret |= VL53L0X_SetVcselPulsePeriod(vl,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, DEFAULT_VCSEL_FINAL_RANGE);
	if (ret) {
		LOG_ERR("[%s] default config failed: %d", dev->name, ret);
		return -EIO;
	}

	return 0;
}

static int vl53l0x_enh_start(const struct device *dev)
{
	const struct vl53l0x_enh_config *cfg = dev->config;
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_DEV vl = &data->vl53l0x;
	VL53L0X_Error ret;
	uint16_t chip_id = 0;
	int r;

	LOG_DBG("[%s] Starting", dev->name);

	/* Release XSHUT if available */
	if (cfg->xshut.port) {
		r = gpio_pin_configure_dt(&cfg->xshut, GPIO_OUTPUT_INACTIVE);
		if (r < 0) {
			LOG_ERR("[%s] XSHUT inactive failed: %d", dev->name, r);
			return -EIO;
		}
		k_sleep(T_BOOT);
	}

#ifdef CONFIG_VL53L0X_ENHANCED_RECONFIGURE_ADDRESS
	if (cfg->i2c.addr != VL53L0X_INITIAL_ADDR) {
		ret = VL53L0X_SetDeviceAddress(vl, 2 * cfg->i2c.addr);
		if (ret) {
			LOG_ERR("[%s] SetDeviceAddress failed", dev->name);
			return -EIO;
		}
		vl->I2cDevAddr = cfg->i2c.addr;
		LOG_DBG("[%s] address reconfigured to 0x%02x",
			dev->name, cfg->i2c.addr);
		k_sleep(T_BOOT);
	}
#endif

	/* Verify chip ID */
	ret = VL53L0X_RdWord(vl, VL53L0X_REG_WHO_AM_I, &chip_id);
	if (ret || chip_id != VL53L0X_CHIP_ID) {
		LOG_ERR("[%s] chip ID mismatch: 0x%04x", dev->name, chip_id);
		return -ENOTSUP;
	}

	/* Data init */
	ret = VL53L0X_DataInit(vl);
	if (ret) {
		LOG_ERR("[%s] DataInit failed: %d", dev->name, ret);
		return -ENOTSUP;
	}

	/* Full setup: calibration + default config */
	r = vl53l0x_enh_setup(dev);
	if (r < 0) {
		return r;
	}

	data->started = true;
	LOG_DBG("[%s] Started", dev->name);
	return 0;
}

/* ─── Sensor API: sample_fetch ───────────────────────────────────────────── */

static int vl53l0x_enh_sample_fetch(const struct device *dev,
				    enum sensor_channel chan)
{
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_DEV vl = &data->vl53l0x;
	VL53L0X_Error ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_DISTANCE ||
			chan == SENSOR_CHAN_PROX);

	if (!data->started) {
		int r = vl53l0x_enh_start(dev);
		if (r < 0) {
			return r;
		}
	}

	if (data->current_mode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
		/* Blocking single-shot measurement */
		ret = VL53L0X_PerformSingleRangingMeasurement(vl,
			&data->ranging_data);
		if (ret) {
			LOG_ERR("[%s] single measurement failed: %d",
				dev->name, ret);
			return -EINVAL;
		}
	} else {
		/* Non-blocking: check if data is ready */
		uint8_t ready = 0;
		ret = VL53L0X_GetMeasurementDataReady(vl, &ready);
		if (ret) {
			return -EIO;
		}
		if (!ready) {
			return -EAGAIN;
		}

		ret = VL53L0X_GetRangingMeasurementData(vl, &data->ranging_data);
		if (ret) {
			return -EIO;
		}

		ret = VL53L0X_ClearInterruptMask(vl, 0);
		if (ret) {
			LOG_WRN("[%s] ClearInterrupt failed: %d",
				dev->name, ret);
		}
	}

	return 0;
}

/* ─── Sensor API: channel_get ────────────────────────────────────────────── */

static int vl53l0x_enh_channel_get(const struct device *dev,
				   enum sensor_channel chan,
				   struct sensor_value *val)
{
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_RangingMeasurementData_t *rd = &data->ranging_data;

	switch ((int)chan) {
	case SENSOR_CHAN_PROX:
		val->val1 = (rd->RangeMilliMeter <=
			     CONFIG_VL53L0X_ENHANCED_PROXIMITY_THRESHOLD) ? 1 : 0;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_DISTANCE:
		val->val1 = rd->RangeMilliMeter / 1000;
		val->val2 = (rd->RangeMilliMeter % 1000) * 1000;
		break;

	case SENSOR_CHAN_VL53L0X_RANGE_STATUS:
		val->val1 = rd->RangeStatus;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_VL53L0X_RANGE_DMAX:
		val->val1 = rd->RangeDMaxMilliMeter / 1000;
		val->val2 = (rd->RangeDMaxMilliMeter % 1000) * 1000;
		break;

	case SENSOR_CHAN_VL53L0X_SIGNAL_RATE_RTN_CPS:
		val->val1 = rd->SignalRateRtnMegaCps >> 16;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_VL53L0X_AMBIENT_RATE_RTN_CPS:
		val->val1 = rd->AmbientRateRtnMegaCps >> 16;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_VL53L0X_EFFECTIVE_SPAD_RTN_COUNT:
		val->val1 = rd->EffectiveSpadRtnCount / 256;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/* ─── Sensor API: attr_set ───────────────────────────────────────────────── */

static int vl53l0x_enh_attr_set(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value *val)
{
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_DEV vl = &data->vl53l0x;
	VL53L0X_Error ret;

	if (!data->started) {
		return -ENODEV;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_VL53L0X_MODE: {
		/* Stop current continuous measurement if running */
		if (data->continuous_running) {
			VL53L0X_StopMeasurement(vl);
			data->continuous_running = false;
		}

		VL53L0X_DeviceModes mode = (VL53L0X_DeviceModes)val->val1;
		ret = VL53L0X_SetDeviceMode(vl, mode);
		if (ret) {
			LOG_ERR("[%s] SetDeviceMode(%d) failed: %d",
				dev->name, mode, ret);
			return -EIO;
		}
		data->current_mode = mode;

		/* Start measurement for continuous modes */
		if (mode != VL53L0X_DEVICEMODE_SINGLE_RANGING) {
			ret = VL53L0X_StartMeasurement(vl);
			if (ret) {
				LOG_ERR("[%s] StartMeasurement failed: %d",
					dev->name, ret);
				return -EIO;
			}
			data->continuous_running = true;
		}
		break;
	}

	case SENSOR_ATTR_VL53L0X_TIMING_BUDGET:
		ret = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl,
			(uint32_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_INTER_MEASUREMENT_PERIOD:
		ret = VL53L0X_SetInterMeasurementPeriodMilliSeconds(vl,
			(uint32_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_VCSEL_PRE_RANGE:
		ret = VL53L0X_SetVcselPulsePeriod(vl,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, (uint8_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_VCSEL_FINAL_RANGE:
		ret = VL53L0X_SetVcselPulsePeriod(vl,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, (uint8_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_SIGNAL_RATE_LIMIT:
		ret = VL53L0X_SetLimitCheckValue(vl,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_SIGMA_LIMIT:
		ret = VL53L0X_SetLimitCheckValue(vl,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			(FixPoint1616_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_PROFILE:
		return apply_profile(data, (enum vl53l0x_profile)val->val1);

	case SENSOR_ATTR_VL53L0X_OFFSET_CAL:
		ret = VL53L0X_SetOffsetCalibrationDataMicroMeter(vl,
			(int32_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	case SENSOR_ATTR_VL53L0X_XTALK_RATE:
		ret = VL53L0X_SetXTalkCompensationRateMegaCps(vl,
			(FixPoint1616_t)val->val1);
		if (ret) {
			return -EIO;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/* ─── Sensor API: attr_get ───────────────────────────────────────────────── */

static int vl53l0x_enh_attr_get(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				struct sensor_value *val)
{
	struct vl53l0x_enh_data *data = dev->data;
	VL53L0X_DEV vl = &data->vl53l0x;
	VL53L0X_Error ret;

	if (!data->started) {
		return -ENODEV;
	}

	val->val2 = 0;

	switch ((int)attr) {
	case SENSOR_ATTR_VL53L0X_MODE:
		val->val1 = (int32_t)data->current_mode;
		break;

	case SENSOR_ATTR_VL53L0X_TIMING_BUDGET: {
		uint32_t budget;
		ret = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(vl, &budget);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)budget;
		break;
	}

	case SENSOR_ATTR_VL53L0X_INTER_MEASUREMENT_PERIOD: {
		uint32_t period;
		ret = VL53L0X_GetInterMeasurementPeriodMilliSeconds(vl, &period);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)period;
		break;
	}

	case SENSOR_ATTR_VL53L0X_VCSEL_PRE_RANGE: {
		uint8_t period;
		ret = VL53L0X_GetVcselPulsePeriod(vl,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, &period);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)period;
		break;
	}

	case SENSOR_ATTR_VL53L0X_VCSEL_FINAL_RANGE: {
		uint8_t period;
		ret = VL53L0X_GetVcselPulsePeriod(vl,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &period);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)period;
		break;
	}

	case SENSOR_ATTR_VL53L0X_SIGNAL_RATE_LIMIT: {
		FixPoint1616_t limit;
		ret = VL53L0X_GetLimitCheckValue(vl,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &limit);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)limit;
		break;
	}

	case SENSOR_ATTR_VL53L0X_SIGMA_LIMIT: {
		FixPoint1616_t limit;
		ret = VL53L0X_GetLimitCheckValue(vl,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, &limit);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)limit;
		break;
	}

	case SENSOR_ATTR_VL53L0X_OFFSET_CAL: {
		int32_t offset;
		ret = VL53L0X_GetOffsetCalibrationDataMicroMeter(vl, &offset);
		if (ret) {
			return -EIO;
		}
		val->val1 = offset;
		break;
	}

	case SENSOR_ATTR_VL53L0X_XTALK_RATE: {
		FixPoint1616_t rate;
		ret = VL53L0X_GetXTalkCompensationRateMegaCps(vl, &rate);
		if (ret) {
			return -EIO;
		}
		val->val1 = (int32_t)rate;
		break;
	}

	default:
		return -ENOTSUP;
	}

	return 0;
}

/* ─── Driver API table ───────────────────────────────────────────────────── */

static DEVICE_API(sensor, vl53l0x_enh_api) = {
	.sample_fetch = vl53l0x_enh_sample_fetch,
	.channel_get = vl53l0x_enh_channel_get,
	.attr_set = vl53l0x_enh_attr_set,
	.attr_get = vl53l0x_enh_attr_get,
};

/* ─── Device init (runs at POST_KERNEL) ──────────────────────────────────── */

static int vl53l0x_enh_init(const struct device *dev)
{
	struct vl53l0x_enh_data *data = dev->data;
	const struct vl53l0x_enh_config *cfg = dev->config;
	int r;

	data->vl53l0x.I2cDevAddr = VL53L0X_INITIAL_ADDR;
	data->vl53l0x.i2c = cfg->i2c.bus;
	data->started = false;
	data->continuous_running = false;
	data->current_mode = VL53L0X_DEVICEMODE_SINGLE_RANGING;

#ifdef CONFIG_VL53L0X_ENHANCED_RECONFIGURE_ADDRESS
	if (!cfg->xshut.port) {
		LOG_ERR("[%s] Missing XSHUT gpio", dev->name);
		return -ENOTSUP;
	}
	/* Shutdown sensor — will be released on first sample_fetch */
	r = gpio_pin_configure_dt(&cfg->xshut, GPIO_OUTPUT_ACTIVE);
	if (r < 0) {
		LOG_ERR("[%s] XSHUT shutdown failed: %d", dev->name, r);
		return -EIO;
	}
	LOG_DBG("[%s] shutdown (XSHUT active)", dev->name);
#else
	if (cfg->i2c.addr != VL53L0X_INITIAL_ADDR) {
		LOG_ERR("[%s] non-default addr requires RECONFIGURE_ADDRESS",
			dev->name);
		return -ENOTSUP;
	}
	r = vl53l0x_enh_start(dev);
	if (r) {
		return r;
	}
#endif

	LOG_DBG("[%s] initialized", dev->name);
	return 0;
}

/* ─── Device instantiation ───────────────────────────────────────────────── */

#define VL53L0X_ENH_INIT(inst)                                               \
	static struct vl53l0x_enh_config vl53l0x_enh_cfg_##inst = {          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                          \
		.xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {}),   \
	};                                                                   \
	static struct vl53l0x_enh_data vl53l0x_enh_data_##inst;              \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, vl53l0x_enh_init,                 \
			      NULL,                                          \
			      &vl53l0x_enh_data_##inst,                      \
			      &vl53l0x_enh_cfg_##inst,                       \
			      POST_KERNEL,                                   \
			      CONFIG_SENSOR_INIT_PRIORITY,                   \
			      &vl53l0x_enh_api);

DT_INST_FOREACH_STATUS_OKAY(VL53L0X_ENH_INIT)
