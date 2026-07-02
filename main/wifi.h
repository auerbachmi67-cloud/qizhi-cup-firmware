#pragma once
#include <stdbool.h>

void wifi_init(void);
void wifi_send_telemetry(float line_pos, float left_speed, float right_speed,
                         float target_speed, float gyro_z,
                         float ff_omega, bool running);
