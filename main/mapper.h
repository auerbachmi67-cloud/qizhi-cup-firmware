#pragma once
#include "ringbuf.h"
#include <stdint.h>

#define MAP_MAX_SAMPLES 1024

typedef struct {
    float distance_m;
    float line_pos;
    float curvature;
    float speed_mps;
} map_sample_t;

void mapper_init(void);
void mapper_sample(void);
void mapper_predict(void);
control_cmd_t mapper_get_command(void);

int32_t mapper_get_count(void);
const map_sample_t* mapper_get_samples(void);
void mapper_reset(void);
int mapper_get_lap(void);
float mapper_get_total_distance(void);

// Load a speed profile from flash/PC for replay on lap 2+
void mapper_load_profile(const float *speed_table, int count);
