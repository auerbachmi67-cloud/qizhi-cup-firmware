#include "line_control.h"
static pid_t line_pid;
void line_control_init(void) { pid_init(&line_pid, 2.0f, 0.2f, 0.1f, 1.0f); }
float line_control_update(float p) { return pid_update(&line_pid, 0.0f, -p); }
