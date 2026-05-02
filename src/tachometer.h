#pragma once

#include <stdint.h>

/* ─── Tachometer: optical encoder on GP13 (RISING edge interrupt) ─────────── */

void taho_init(void);

/* Get current speed in m/s (interval-based, like Arduino get_speed()) */
float taho_get_speed(void);

/* Get raw pulse count (atomic) */
uint32_t taho_get_count(void);

/* Get µs since last pulse */
uint32_t taho_time_since_last_us(void);

/* Reset counters (for tests) */
void taho_reset(void);

/* Runtime update of ISR glitch filter threshold (µs) */
void taho_set_glitch_filter_us(uint32_t us);
