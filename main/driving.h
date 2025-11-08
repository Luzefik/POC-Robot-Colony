#pragma once
#include "esp_err.h"

void motor_init(void);

void motor_set_speed(int motor_id, float speed_percent);

void motor_stop(int motor_id);

void stop_all(void);
