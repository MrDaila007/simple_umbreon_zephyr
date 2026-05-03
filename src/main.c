/*
 * main.c — Simple umbreon racing firmware entry point.
 *
 * Stripped from umbreon_zephyr: no IMU, battery, display, encoder,
 * track_learn, or tests. Control thread runs the wall-follow law directly.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_USB_DEVICE_STACK)
#include <zephyr/usb/usb_device.h>
#endif

#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "battery.h"
#include "wifi_cmd.h"
#include "control.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define FW_VERSION "1.0.0"

/* ─── Watchdog ──────────────────────────────────────────────────────────────── */
#include <zephyr/drivers/watchdog.h>

static const struct device *wdt_dev;
static int wdt_channel_id;

static void wdt_init(void)
{
	wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("Watchdog not available");
		wdt_dev = NULL;
		return;
	}

	struct wdt_timeout_cfg wdt_cfg = {
		.window.min = 0,
		.window.max = 8000,
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install failed: %d", wdt_channel_id);
		wdt_dev = NULL;
		return;
	}

	int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err) {
		LOG_ERR("Watchdog setup failed: %d", err);
		wdt_dev = NULL;
	}
}

void wdt_feed_kick(void)
{
	if (wdt_dev) {
		wdt_feed(wdt_dev, wdt_channel_id);
	}
}

/* ─── LED blink ──────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void blink_led(int count, int ms)
{
	if (!gpio_is_ready_dt(&led)) {
		return;
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	for (int i = 0; i < count; i++) {
		gpio_pin_set_dt(&led, 1);
		k_msleep(ms);
		gpio_pin_set_dt(&led, 0);
		k_msleep(ms);
	}
}

/* ─── USB console init ───────────────────────────────────────────────────────── */
/* Wait up to 3 s for a USB terminal to assert DTR before printing the banner.
 * Works for both the legacy USB_DEVICE_STACK and the newer _NEXT stack. */
static void usb_console_init(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(dev)) {
		return;
	}

#if defined(CONFIG_USB_DEVICE_STACK)
	usb_enable(NULL);
#endif

	uint32_t dtr = 0;
	int64_t deadline = k_uptime_get() + 3000;

	while (!dtr && k_uptime_get() < deadline) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_msleep(50);
	}
}

/* ─── Main ──────────────────────────────────────────────────────────────────── */
int main(void)
{
	usb_console_init();

	printk("\n");
	printk("==============================\n");
	printk("  Umbreon Simple v%s\n", FW_VERSION);
	printk("==============================\n");

	settings_init();
	settings_load();

	wdt_init();

	car_init();
	taho_init();
	sensors_init();      /* ~500 ms I2C probing */
	battery_init();
	wdt_feed_kick();
	wifi_cmd_init();

	k_msleep(200);
	wifi_cmd_printf("$BOOT:SNS=%d,FW=%s\n",
			sensors_online_count(), FW_VERSION);

	wdt_feed_kick();
	if (!cfg.calibrated) {
		car_run_calibration();
	} else {
		k_msleep(3700);
	}

	control_init();

	blink_led(3, 100);

	wifi_cmd_printf("$BOOT:READY,UP=%lld\n", k_uptime_get());
	wifi_cmd_send_uicap();

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
