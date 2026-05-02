#pragma once

#include <stdbool.h>

/* ─── ADC battery voltage monitor (GP28 / channel 2) ─────────────────────── */

void  battery_init(void);

/* Read battery voltage in volts. Returns 0 if bat_enabled=false or ADC not
 * ready. Result is cached; safe to call from multiple threads. */
float battery_voltage(void);

/* True when bat_enabled=true and voltage < bat_low threshold. */
bool  battery_is_low(void);
