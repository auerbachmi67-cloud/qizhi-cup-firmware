#include "line_control.h"
#include "config.h"

static pid_ctrl_t line_pid;
static float last_line = 0.0f;

void line_control_init(void) {
    pid_init(&line_pid, LINE_PID_KP, LINE_PID_KI, LINE_PID_KD, PID_OUTPUT_LIMIT);
}

// Legacy — used on mapping lap (no FF)
float line_control_update(float line_position) {
    last_line = line_position;
    return pid_update(&line_pid, 0.0f, -line_position);
}

// FF mode: ω_ff (°/s) → throttle diff via experimental gain
// Theory: diff = ω_dps × 0.000443, but we use tunable FF_OMEGA_GAIN
float line_control_update_ff(float line_position, float gyro_ff, float speed_mps) {
    last_line = line_position;
    (void)speed_mps;  // reserved for speed-dependent gain scaling

    // PID feedback — reduced gain when FF is active
    float fb = pid_update(&line_pid, 0.0f, -line_position) * LINE_PID_FF_RATIO;

    // Feedforward: gyro (°/s) → throttle differential
    // sign convention: positive gyro = CCW (left turn) → right wheel faster
    float ff = gyro_ff * FF_OMEGA_GAIN;

    return fb + ff;
}

// Lost-line emergency turn — steer hard in last known direction
float line_control_search_turn(float last_known_side) {
    // Reset PID state so integral doesn't carry over from before loss
    pid_reset(&line_pid);
    return (last_known_side > 0) ? 0.5f : -0.5f;  // half throttle diff to search
}
