#pragma once
#include "ringbuf.h"
#include <stdint.h>
#include <stdbool.h>

#define MAP_MAX_SAMPLES 2048
#define MAP_SPACING_MM   5     // spatial sampling interval

typedef struct {
    float distance_m;
    float gyro_z;           // recorded gyro rate (°/s) — replaces κ
    float line_pos;
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

// Load full target profile: (gyro_ff, target_speed) pairs, interleaved
void mapper_load_profile(const float *data, int count);
