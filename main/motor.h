#pragma once
#include <stdint.h>

void motor_init(void);
void motor_set_speed(int channel, float throttle);
void motor_brake(int channel);
void motor_coast(void);
