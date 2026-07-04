#include "mapper.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MAP";
static map_sample_t map[MAP_MAX_SAMPLES];
static int32_t map_count = 0;
static float total_dist = 0.0f;
static float lap_reference_length = 0.0f;
static int lap = 0;

static float *profile_data = NULL;
static int profile_pairs = 0;

static float last_sample_dist = 0.0f;
static control_cmd_t next_cmd = {false, 0, MAX_SPEED_MPS, 0};

void mapper_init(void) {
    memset(map, 0, sizeof(map)); map_count = 0; total_dist = 0.0f;
    lap_reference_length = 0.0f; lap = 0; last_sample_dist = 0.0f;
    next_cmd.active = false; profile_data = NULL; profile_pairs = 0;
    ESP_LOGI(TAG, "Mapper reset (gyro-FF mode)");
}

void mapper_load_profile(const float *data, int count) {
    if (count < 4 || (count & 1)) {
        ESP_LOGE(TAG, "Invalid profile: %d floats", count);
        return;
    }
    profile_data = (float*)data;
    profile_pairs = count / 2;
    lap = 1;
    ESP_LOGI(TAG, "Profile loaded: %d pairs", profile_pairs);
}

void mapper_sample(float avg_spd, float line_pos, float gyro_z) {
    if (map_count >= MAP_MAX_SAMPLES) return;

    total_dist += avg_spd / (float)MAP_SAMPLE_RATE_HZ;

    if (total_dist - last_sample_dist < (MAP_SPACING_MM / 1000.0f)) return;
    last_sample_dist = total_dist;

    map[map_count].distance_m = total_dist;
    map[map_count].gyro_z     = gyro_z;
    map[map_count].line_pos   = line_pos;
    map[map_count].speed_mps  = avg_spd;
    map_count++;
}

void mapper_predict(void) {
    if (map_count < 2) { next_cmd.active = false; return; }

    if (profile_data && profile_pairs > 0) {
        if (lap_reference_length < 0.001f) { next_cmd.active = false; return; }

        float ratio = total_dist / lap_reference_length;
        if (ratio > 1.05f) { next_cmd.active = false; return; }
        if (ratio < 0.0f)  ratio = 0.0f;

        float fidx = ratio * (float)(profile_pairs - 1);
        int idx = (int)fidx;
        if (idx >= profile_pairs - 1) idx = profile_pairs - 2;
        if (idx < 0) idx = 0;

        float frac = fidx - (float)idx;
        float gyro_ff_a = profile_data[idx * 2];
        float speed_a   = profile_data[idx * 2 + 1];
        float gyro_ff_b = profile_data[(idx + 1) * 2];
        float speed_b   = profile_data[(idx + 1) * 2 + 1];

        next_cmd.active           = true;
        next_cmd.feedforward_omega = gyro_ff_a + (gyro_ff_b - gyro_ff_a) * frac;
        next_cmd.target_speed      = speed_a + (speed_b - speed_a) * frac;
        next_cmd.timestamp_ms      = (uint32_t)(esp_timer_get_time() / 1000);
        return;
    }

    float lookahead = total_dist + 0.1f;
    int idx = 0; float best_d = 1e9f;
    for (int i = 0; i < map_count; i++) {
        float d = fabsf(map[i].distance_m - lookahead);
        if (d < best_d) { best_d = d; idx = i; }
    }

    float gyro_abs = fabsf(map[idx].gyro_z);
    float omega_rad = gyro_abs * 0.0174533f;
    float v_est = (fabsf(map[idx].speed_mps) > 0.05f) ? fabsf(map[idx].speed_mps) : MAX_SPEED_MPS;
    float k = omega_rad / v_est;
    if (k < 0.01f) k = 0.01f;
    float v_max = sqrtf(3.924f / k);
    if (v_max > MAX_SPEED_MPS) v_max = MAX_SPEED_MPS;

    next_cmd.active = true;
    next_cmd.target_speed = v_max;
    next_cmd.feedforward_omega = 0;
    next_cmd.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

control_cmd_t mapper_get_command(void) { return next_cmd; }
int32_t mapper_get_count(void) { return map_count; }
const map_sample_t* mapper_get_samples(void) { return map; }

void mapper_reset(void) {
    if (lap == 0 && map_count > 0) {
        lap_reference_length = total_dist;
        ESP_LOGI(TAG, "Lap 0 reference length: %.3f m (%ld samples)",
                 lap_reference_length, (long)map_count);
    }
    map_count = 0; total_dist = 0.0f; last_sample_dist = 0.0f; lap++;
}

int mapper_get_lap(void) { return lap; }
float mapper_get_total_distance(void) { return total_dist; }
