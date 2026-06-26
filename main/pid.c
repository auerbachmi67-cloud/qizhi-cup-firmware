#include "pid.h"
void pid_init(pid_t *pid, float kp, float ki, float kd, float limit) {
    pid->kp=kp; pid->ki=ki; pid->kd=kd; pid->integral=0; pid->prev_error=0; pid->output_limit=limit;
}
float pid_update(pid_t *pid, float sp, float meas) {
    float e = sp - meas;
    pid->integral += e;
    float il = pid->output_limit / (pid->ki > 0.001f ? pid->ki : 0.001f);
    if (pid->integral > il) pid->integral = il;
    if (pid->integral < -il) pid->integral = -il;
    float d = e - pid->prev_error; pid->prev_error = e;
    float o = pid->kp * e + pid->ki * pid->integral + pid->kd * d;
    if (o > pid->output_limit) o = pid->output_limit;
    if (o < -pid->output_limit) o = -pid->output_limit;
    return o;
}
float pid_update_hold(pid_t *pid, float sp, float meas) {
    float e = sp - meas;
    // Keep integral frozen during saturation
    float d = e - pid->prev_error; pid->prev_error = e;
    float o = pid->kp * e + pid->ki * pid->integral + pid->kd * d;
    if (o > pid->output_limit) o = pid->output_limit;
    if (o < -pid->output_limit) o = -pid->output_limit;
    return o;
}
void pid_reset(pid_t *pid) { pid->integral = 0; pid->prev_error = 0; }
