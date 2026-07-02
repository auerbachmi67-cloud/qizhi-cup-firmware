#pragma once
#include "pid.h"

void line_control_init(void);
float line_control_update(float line_position);

// Inject gyro feedforward (°/s) — added to PID output to form total diff
float line_control_update_ff(float line_position, float gyro_ff, float speed_mps);

// Bang-bang search: steer full while line is lost
float line_control_search_turn(float last_known_side);
