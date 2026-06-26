#include "speed_control.h"
#include "pid.h"
static pid_t speed_pid;
static bool hold = false;
void speed_control_init(void) { pid_init(&speed_pid, 1.5f, 0.1f, 0.05f, 1.0f); }
float speed_control_update(float t, float c) { return hold ? pid_update_hold(&speed_pid, t, c) : pid_update(&speed_pid, t, c); }
void speed_hold_integral(void) { hold = true; }
void speed_release_integral(void) { hold = false; }
