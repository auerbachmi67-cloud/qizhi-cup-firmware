#pragma once
#include <stdint.h>
void encoder_init(void);
void encoder_update(void);
float encoder_get_speed(int channel);
int32_t encoder_get_count(int channel);
void encoder_reset(int channel);
