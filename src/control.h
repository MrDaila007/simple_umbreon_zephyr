#pragma once

#include <stdbool.h>

void control_init(void);

bool control_is_running(void);

void control_cmd_start(void);
void control_cmd_stop(void);

bool control_drive_enabled(void);
bool control_set_manual(int steer, float speed);
bool control_set_raw_esc_us(int us);
bool control_set_raw_servo(int angle);
void control_set_drv_enabled(bool enabled);
