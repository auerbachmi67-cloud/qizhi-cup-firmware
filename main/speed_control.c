#include "speed_control.h"
#include "pid.h"
#include "config.h"

static pid_t speed_pid;
static bool hold = false;

void speed_control_init(void) {
    pid_init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 1.0f);
}

float speed_control_update(float t, float c) {
    return hold ? pid_update_hold(&speed_pid, t, c) : pid_update(&speed_pid, t, c);
}

void speed_hold_integral(void) { hold = true; }
void speed_release_integral(void) { hold = false; }
void speed_reset(void) { pid_reset(&speed_pid); hold = false; }
