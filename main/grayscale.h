#pragma once
#include <stdbool.h>
#include <stdint.h>

void grayscale_init(void);
void grayscale_calibrate(void);
float grayscale_get_line_position(void);
uint8_t grayscale_get_digital(void);
bool grayscale_is_ok(void);
