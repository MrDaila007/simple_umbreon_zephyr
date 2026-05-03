/*
 * wifi_cmd.c — UART1 WiFi command protocol (minimal racing set)
 *
 * Supports: $PING $UICAP $GET $SET $SAVE $LOAD $RST $START $STOP $STATUS
 *           $BAT $DRV $DRVEN $DRVOFF $SRV $ESC $LOG:ON $LOG:OFF
 * Drops from umbreon_zephyr: $TEST $TRK $MONITOR $DIAG $SNS $IMU $PID $SYS $HELP
 * Drops: async-cmd thread, menu_cmd_q poll branch.
 */

#include "wifi_cmd.h"
#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "battery.h"
#include "control.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

LOG_MODULE_REGISTER(wifi_cmd, LOG_LEVEL_INF);

#define UI_MANIFEST \
	"$UI:sec=sensors,run,ctrl,settings,drive,console\n"

/* ─── UART device ─────────────────────────────────────────────────────────── */
static const struct device *uart_dev;

/* ─── Ring buffer for UART RX ─────────────────────────────────────────────── */
#define RX_BUF_SIZE 512
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

/* Semaphore: ISR posts when '\n' received */
static K_SEM_DEFINE(rx_line_sem, 0, 1);

/* ─── Command line buffer ─────────────────────────────────────────────────── */
#define CMD_BUF_SIZE 512
static char cmd_buf[CMD_BUF_SIZE];
static int cmd_len;

/* ─── Async TX ring buffer ───────────────────────────────────────────────── */
#define TX_BUF_SIZE 1024
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static K_MUTEX_DEFINE(tx_mutex);

/* ─── TX overflow ring buffer ───────────────────────────────────────────── */
#define OVF_BUF_SIZE 512
static uint8_t ovf_buf[OVF_BUF_SIZE];
static volatile uint16_t ovf_head;
static volatile uint16_t ovf_tail;

/* ─── Debug log flag ──────────────────────────────────────────────────────── */
static volatile bool log_on;

/* ─── Thread ──────────────────────────────────────────────────────────────── */
#define WIFI_STACK_SIZE 2048
#define WIFI_PRIORITY   5
static K_THREAD_STACK_DEFINE(wifi_stack, WIFI_STACK_SIZE);
static struct k_thread wifi_thread_data;

static void sync_tach_glitch_filter(void)
{
	struct car_settings c;
	settings_get_copy(&c);
	taho_set_glitch_filter_us((uint32_t)c.tach_glitch_filter_us);
}

/* ─── TX helpers ──────────────────────────────────────────────────────────── */

void wifi_cmd_send(const char *str)
{
	if (!uart_dev) {
		return;
	}

	k_mutex_lock(&tx_mutex, K_FOREVER);

	bool use_ovf = (ovf_head != ovf_tail);

	for (const char *p = str; *p; p++) {
		if (use_ovf) {
			uint16_t onext = (ovf_head + 1) % OVF_BUF_SIZE;
			if (onext == ovf_tail) {
				break;
			}
			ovf_buf[ovf_head] = (uint8_t)*p;
			ovf_head = onext;
		} else {
			uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
			if (next == tx_tail) {
				use_ovf = true;
				uint16_t onext = (ovf_head + 1) % OVF_BUF_SIZE;
				if (onext == ovf_tail) {
					break;
				}
				ovf_buf[ovf_head] = (uint8_t)*p;
				ovf_head = onext;
			} else {
				tx_buf[tx_head] = (uint8_t)*p;
				tx_head = next;
			}
		}
	}

	uart_irq_tx_enable(uart_dev);
	k_mutex_unlock(&tx_mutex);
}

void wifi_cmd_printf(const char *fmt, ...)
{
	char buf[384];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	wifi_cmd_send(buf);
}

void wifi_log(const char *fmt, ...)
{
	if (!log_on || !uart_dev) {
		return;
	}
	char buf[256];
	int n = snprintf(buf, sizeof(buf), "$L:");
	va_list args;
	va_start(args, fmt);
	n += vsnprintf(buf + n, sizeof(buf) - n, fmt, args);
	va_end(args);
	if (n > 0 && n < (int)sizeof(buf) - 1 && buf[n - 1] != '\n') {
		buf[n++] = '\n';
		buf[n] = '\0';
	}
	wifi_cmd_send(buf);
}

bool wifi_log_enabled(void)
{
	return log_on;
}

/* ─── Ring buffer helpers ─────────────────────────────────────────────────── */

static inline bool rb_empty(void)
{
	return rx_head == rx_tail;
}

static inline bool rb_get(uint8_t *c)
{
	if (rb_empty()) {
		return false;
	}
	*c = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
	return true;
}

/* ─── UART ISR ────────────────────────────────────────────────────────────── */

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t c;
		int len = uart_fifo_read(dev, &c, 1);
		if (len != 1) {
			continue;
		}

		uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
		if (next != rx_tail) {
			rx_buf[rx_head] = c;
			rx_head = next;
		}

		if (c == '\n') {
			k_sem_give(&rx_line_sem);
		}
	}

	if (uart_irq_tx_ready(dev)) {
		uint16_t t = tx_tail;
		uint16_t h = tx_head;

		if (t != h) {
			uint16_t len = (h > t) ? (h - t) : (TX_BUF_SIZE - t);
			int sent = uart_fifo_fill(dev, &tx_buf[t], len);
			if (sent > 0) {
				tx_tail = (t + sent) % TX_BUF_SIZE;
			}
		} else {
			uint16_t ot = ovf_tail;
			uint16_t oh = ovf_head;
			if (ot != oh) {
				uint16_t len = (oh > ot) ? (oh - ot) : (OVF_BUF_SIZE - ot);
				int sent = uart_fifo_fill(dev, &ovf_buf[ot], len);
				if (sent > 0) {
					ovf_tail = (ot + sent) % OVF_BUF_SIZE;
				}
			} else {
				uart_irq_tx_disable(dev);
			}
		}
	}
}

/* ─── SET command parser ──────────────────────────────────────────────────── */

static bool parse_set_pair(const char *pair)
{
	const char *eq = strchr(pair, '=');
	if (!eq) {
		return false;
	}

	char key[8];
	int klen = eq - pair;
	if (klen <= 0 || klen >= (int)sizeof(key)) {
		return false;
	}
	memcpy(key, pair, klen);
	key[klen] = '\0';
	const char *val = eq + 1;

	if      (strcmp(key, "FOD")  == 0) cfg.front_obstacle_dist = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "SOD")  == 0) cfg.side_open_dist      = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "ACD")  == 0) cfg.all_close_dist      = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "CFD")  == 0) cfg.close_front_dist    = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "KP")   == 0) cfg.pid_kp              = strtof(val, NULL);
	else if (strcmp(key, "KI")   == 0) cfg.pid_ki              = strtof(val, NULL);
	else if (strcmp(key, "KD")   == 0) cfg.pid_kd              = strtof(val, NULL);
	else if (strcmp(key, "MSP")  == 0) cfg.min_speed           = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "XSP")  == 0) cfg.max_speed           = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "BSP")  == 0) cfg.min_bspeed          = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "MNP")  == 0) cfg.min_point           = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "XNP")  == 0) cfg.max_point           = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "NTP")  == 0) cfg.neutral_point       = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "ENH")  == 0) cfg.encoder_holes       = MAX(atoi(val), 1);
	else if (strcmp(key, "WDM")  == 0) cfg.wheel_diam_m        = strtof(val, NULL);
	else if (strcmp(key, "LMS")  == 0) cfg.loop_ms             = MAX(atoi(val), 10);
	else if (strcmp(key, "SPD1") == 0) cfg.spd_clear           = strtof(val, NULL);
	else if (strcmp(key, "SPD2") == 0) cfg.spd_blocked         = strtof(val, NULL);
	else if (strcmp(key, "SLW")  == 0) cfg.spd_slew            = strtof(val, NULL);
	else if (strcmp(key, "KOP")  == 0) cfg.kick_pct            = strtof(val, NULL);
	else if (strcmp(key, "KOM")  == 0) cfg.kick_ms             = MAX(atoi(val), 0);
	else if (strcmp(key, "COE1") == 0) cfg.coe_clear           = strtof(val, NULL);
	else if (strcmp(key, "COE2") == 0) cfg.coe_blocked         = strtof(val, NULL);
	else if (strcmp(key, "SVR")  == 0) cfg.servo_reverse         = atoi(val) != 0;
	else if (strcmp(key, "S6")   == 0) cfg.use_six_sensors       = atoi(val) != 0;
	else if (strcmp(key, "CAL")  == 0) cfg.calibrated            = atoi(val) != 0;
	else if (strcmp(key, "TGF")  == 0) cfg.tach_glitch_filter_us = CLAMP(atoi(val), 1, 500);
	else if (strcmp(key, "BEN")  == 0) cfg.bat_enabled           = atoi(val) != 0;
	else if (strcmp(key, "BML")  == 0) cfg.bat_multiplier        = strtof(val, NULL);
	else if (strcmp(key, "BLV")  == 0) cfg.bat_low               = strtof(val, NULL);
	else if (strcmp(key, "RVT")  == 0) cfg.reverse_time_ms       = CLAMP(atoi(val), 0, 5000);
	else if (strcmp(key, "TRT")  == 0) cfg.turn_time_ms          = CLAMP(atoi(val), 0, 5000);
	else if (strcmp(key, "RVS")  == 0) cfg.reverse_speed         = strtof(val, NULL);
	else return false;

	return true;
}

/* ─── GET response ────────────────────────────────────────────────────────── */

static void cmd_get(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	wifi_cmd_printf(
		"$CFG:FOD=%d,SOD=%d,ACD=%d,CFD=%d"
		",KP=%.4f,KI=%.4f,KD=%.4f"
		",MSP=%d,XSP=%d,BSP=%d"
		",MNP=%d,XNP=%d,NTP=%d",
		c.front_obstacle_dist, c.side_open_dist,
		c.all_close_dist, c.close_front_dist,
		(double)c.pid_kp, (double)c.pid_ki, (double)c.pid_kd,
		c.min_speed, c.max_speed, c.min_bspeed,
		c.min_point, c.max_point, c.neutral_point);

	wifi_cmd_printf(
		",ENH=%d,WDM=%.4f,LMS=%d"
		",SPD1=%.1f,SPD2=%.1f,SLW=%.2f,KOP=%.1f,KOM=%d"
		",COE1=%.2f,COE2=%.2f",
		c.encoder_holes, (double)c.wheel_diam_m, c.loop_ms,
		(double)c.spd_clear, (double)c.spd_blocked,
		(double)c.spd_slew,
		(double)c.kick_pct, c.kick_ms,
		(double)c.coe_clear, (double)c.coe_blocked);

	wifi_cmd_printf(
		",SVR=%d,S6=%d,CAL=%d"
		",TGF=%d"
		",BEN=%d,BML=%.2f,BLV=%.1f,BV=%.2f"
		",RVT=%d,TRT=%d,RVS=%.2f"
		",SNS=%d,SMX=%d,FWV=1.0.0\n",
		c.servo_reverse ? 1 : 0,
		c.use_six_sensors ? 1 : 0,
		c.calibrated ? 1 : 0,
		c.tach_glitch_filter_us,
		c.bat_enabled ? 1 : 0,
		(double)c.bat_multiplier, (double)c.bat_low,
		(double)battery_voltage(),
		c.reverse_time_ms, c.turn_time_ms, (double)c.reverse_speed,
		SENSOR_COUNT, MAX_SENSOR_RANGE);
}

/* ─── SET command ─────────────────────────────────────────────────────────── */

static void cmd_set(const char *args)
{
	char buf[CMD_BUF_SIZE];
	strncpy(buf, args, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	settings_lock();
	char *token = strtok(buf, ",");
	while (token) {
		if (!parse_set_pair(token)) {
			settings_unlock();
			wifi_cmd_printf("$NAK:%s\n", token);
			return;
		}
		token = strtok(NULL, ",");
	}
	int glitch_us = cfg.tach_glitch_filter_us;
	settings_unlock();
	taho_set_glitch_filter_us((uint32_t)glitch_us);
	wifi_cmd_send("$ACK\n");
}

/* ─── DRV command ─────────────────────────────────────────────────────────── */

static void cmd_drv(const char *args)
{
	char buf[32];
	strncpy(buf, args, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	char *comma = strchr(buf, ',');
	if (!comma) {
		return;
	}
	*comma = '\0';
	int steer = atoi(buf);
	float speed = strtof(comma + 1, NULL);
	control_set_manual(steer, speed);
}

void wifi_cmd_send_uicap(void) { wifi_cmd_send(UI_MANIFEST); }

/* ─── Command dispatcher ─────────────────────────────────────────────────── */

static void dispatch_command(const char *line)
{
	if (strcmp(line, "$PING") == 0) {
		wifi_cmd_send("$PONG\n");
	} else if (strcmp(line, "$UICAP") == 0) {
		wifi_cmd_send_uicap();
	} else if (strcmp(line, "$GET") == 0) {
		cmd_get();
	} else if (strncmp(line, "$SET:", 5) == 0) {
		cmd_set(line + 5);
	} else if (strcmp(line, "$SAVE") == 0) {
		settings_save();
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$LOAD") == 0) {
		if (settings_load()) {
			sync_tach_glitch_filter();
			wifi_cmd_send("$ACK\n");
		} else {
			wifi_cmd_send("$NAK:no_saved_config\n");
		}
	} else if (strcmp(line, "$RST") == 0) {
		settings_reset();
		sync_tach_glitch_filter();
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$START") == 0) {
		if (control_is_running()) {
			wifi_cmd_send("$NAK:already_running\n");
		} else {
			control_cmd_start();
		}
	} else if (strcmp(line, "$STOP") == 0) {
		control_cmd_stop();
	} else if (strcmp(line, "$STATUS") == 0) {
		wifi_cmd_printf("$STS:%s\n", control_is_running() ? "RUN" : "STOP");
	} else if (strcmp(line, "$BAT") == 0) {
		wifi_cmd_printf("$BAT:%.2f\n", (double)battery_voltage());
	} else if (strncmp(line, "$DRV:", 5) == 0) {
		cmd_drv(line + 5);
	} else if (strncmp(line, "$SRV:", 5) == 0) {
		int angle = CLAMP(atoi(line + 5), 0, 180);
		car_write_servo_raw(angle);
	} else if (strncmp(line, "$ESC:", 5) == 0) {
		int val = CLAMP(atoi(line + 5), 1000, 2000);
		car_write_esc_us(val);
	} else if (strcmp(line, "$DRVEN") == 0) {
		control_set_drv_enabled(true);
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$DRVOFF") == 0) {
		control_set_drv_enabled(false);
		car_write_steer(0);
		car_write_speed(0);
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$LOG:ON") == 0) {
		log_on = true;
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$LOG:OFF") == 0) {
		log_on = false;
		wifi_cmd_send("$ACK\n");
	}
	/* Unknown commands silently ignored */
}

/* ─── Thread entry point ──────────────────────────────────────────────────── */

static void wifi_cmd_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("WiFi command thread started");

	wifi_cmd_send("#ms,s0,s1,s2,s3,s4,s5,steer,speed,target\n");
	wifi_cmd_send("#WIFISTATUS\n");

	while (1) {
		k_sem_take(&rx_line_sem, K_FOREVER);

		uint8_t c;
		while (rb_get(&c)) {
			if (c == '\n' || c == '\r') {
				if (cmd_len > 0) {
					cmd_buf[cmd_len] = '\0';
					if (cmd_buf[0] == '$') {
						dispatch_command(cmd_buf);
					}
					cmd_len = 0;
				}
			} else if (cmd_len < CMD_BUF_SIZE - 1) {
				cmd_buf[cmd_len++] = (char)c;
			}
		}
	}
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void wifi_cmd_init(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART1 not ready");
		return;
	}

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	k_thread_create(&wifi_thread_data, wifi_stack,
			K_THREAD_STACK_SIZEOF(wifi_stack),
			wifi_cmd_thread, NULL, NULL, NULL,
			WIFI_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&wifi_thread_data, "wifi_cmd");

	LOG_INF("WiFi CMD init (UART1 GP4/GP5, 115200)");
}
