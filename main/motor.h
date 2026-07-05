#pragma once
#include <stdint.h>
#include <stdbool.h>

void motor_init(void);
void motor_set_speed(int channel, float throttle);
void motor_brake(int channel);
void motor_coast(void);

// Smooth stop ramp from current throttles (100-step linear deceleration)
void motor_stop_ramp(float cur_left, float cur_right);
bool motor_ramp_active(void);
void motor_ramp_tick(void);

// Battery voltage compensation (scales throttle to normalize speed)
void motor_set_vbat_comp(float comp);
float motor_get_vbat_comp(void);
