#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    bool active;
    float feedforward_omega;
    float target_speed;
    float curvature;
} control_cmd_t;
void ringbuf_init(void);
void ringbuf_push(control_cmd_t cmd);
control_cmd_t ringbuf_pop(void);
bool ringbuf_empty(void);
