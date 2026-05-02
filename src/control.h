#pragma once

#include <stdbool.h>

void control_init(void);

bool control_is_running(void);

void control_cmd_start(void);
void control_cmd_stop(void);

void control_set_manual(int steer, float speed);
void control_set_drv_enabled(bool enabled);
