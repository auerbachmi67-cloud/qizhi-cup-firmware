#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    float kp, ki, kd; float integral, prev_error; float output_limit;
} pid_t;
void pid_init(pid_t *pid, float kp, float ki, float kd, float limit);
float pid_update(pid_t *pid, float setpoint, float measurement);
float pid_update_hold(pid_t *pid, float setpoint, float measurement);
void pid_reset(pid_t *pid);
