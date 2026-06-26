#pragma once
void speed_control_init(void);
float speed_control_update(float target, float current);
void speed_hold_integral(void);
void speed_release_integral(void);
