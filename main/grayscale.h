#pragma once
#include <stdint.h>
#include <stdbool.h>
void grayscale_init(void);
void grayscale_read(uint16_t *values);
void grayscale_calibrate(void);
float grayscale_get_line_position(void);
void grayscale_set_threshold(float t);
bool grayscale_is_calibrated(void);
