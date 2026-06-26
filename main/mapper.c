#include "mapper.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MAP";
static map_sample_t map[MAP_MAX_SAMPLES];
static int32_t map_count = 0;
static float total_dist = 0.0f;
static float lap_reference_length = 0.0f;
static int lap = 0;
static float *profile_speed = NULL;
static int profile_len = 0;
static control_cmd_t next_cmd = {false, 0, MAX_SPEED_MPS, 0};

void mapper_init(void) {
    memset(map, 0, sizeof(map)); map_count = 0; total_dist = 0.0f;
    lap_reference_length = 0.0f; lap = 0;
    next_cmd.active = false; profile_speed = NULL; profile_len = 0;
    ESP_LOGI(TAG, "Mapper reset");
}

void mapper_load_profile(const float *speed_table, int count) {
    profile_speed = (float*)speed_table; profile_len = count; lap = 1;
    ESP_LOGI(TAG, "Profile loaded: %d points", count);
}

void mapper_sample(void) {
    if (map_count >= MAP_MAX_SAMPLES) return;
    float l_spd = encoder_get_speed(0);
    float r_spd = encoder_get_speed(1);
    float avg_spd = (l_spd + r_spd) * 0.5f;
    total_dist += avg_spd / (float)MAP_SAMPLE_RATE_HZ;

    float line_pos = grayscale_get_line_position();
    float gyro_z = imu_get_gyro_z();

    // κ = ω / v  with zero-protection (fix #5)
    float curvature = 0.0f;
    if (fabsf(avg_spd) > 0.05f)
        curvature = (gyro_z * 0.0174533f) / avg_spd; // PI/180

    map[map_count].distance_m = total_dist;
    map[map_count].line_pos = line_pos;
    map[map_count].curvature = curvature;
    map[map_count].speed_mps = avg_spd;
    map_count++;
}

void mapper_predict(void) {
    if (lap == 0 || map_count < 2) { next_cmd.active = false; return; }
    if (lap_reference_length < 0.001f) { next_cmd.active = false; return; }
    if (profile_speed && profile_len > 0) {
        float ratio = total_dist / lap_reference_length; // normalize by lap 0 length
        if (ratio > 1.0f) ratio = 1.0f;
        int idx = (int)(ratio * (float)(profile_len - 1));
        if (idx >= profile_len) idx = profile_len - 1;
        next_cmd.active = true;
        next_cmd.target_speed = profile_speed[idx];
        next_cmd.curvature = map[idx < map_count ? idx : map_count - 1].curvature;
        next_cmd.feedforward_omega = 0;
        return;
    }
    float lookahead = total_dist + 0.1f;
    int idx = 0; float best_d = 1e9f;
    for (int i = 0; i < map_count; i++) {
        float d = fabsf(map[i].distance_m - lookahead);
        if (d < best_d) { best_d = d; idx = i; }
    }
    float k = fabsf(map[idx].curvature);
    float v_max = sqrtf(3.924f / fmaxf(k, 0.01f)); // μg/κ
    if (v_max > MAX_SPEED_MPS) v_max = MAX_SPEED_MPS;
    next_cmd.active = true; next_cmd.target_speed = v_max;
    next_cmd.curvature = map[idx].curvature; next_cmd.feedforward_omega = 0;
}

control_cmd_t mapper_get_command(void) { return next_cmd; }
int32_t mapper_get_count(void) { return map_count; }
const map_sample_t* mapper_get_samples(void) { return map; }
void mapper_reset(void) {
    // Before clearing, save lap 0 length as reference for profile replay
    if (lap == 0 && map_count > 0) {
        lap_reference_length = total_dist;
        ESP_LOGI(TAG, "Lap 0 reference length: %.3f m", lap_reference_length);
    }
    map_count = 0; total_dist = 0.0f; lap++;
}
int mapper_get_lap(void) { return lap; }
float mapper_get_total_distance(void) { return total_dist; }
