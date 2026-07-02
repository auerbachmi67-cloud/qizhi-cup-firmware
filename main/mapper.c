#include "mapper.h"
#include "grayscale.h"
#include "imu.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
extern float encoder_get_speed(int ch);

static const char *TAG = "MAP";
static map_sample_t map[MAP_MAX_SAMPLES];
static int32_t map_count = 0;
static float total_dist = 0.0f;
static float lap_reference_length = 0.0f;
static int lap = 0;

// Profile storage: interleaved (gyro_ff, target_speed) pairs
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
    // data: interleaved (gyro_ff, target_speed) pairs, so count is total floats
    // count must be even (2 floats per pair)
    if (count < 4 || (count & 1)) {
        ESP_LOGE(TAG, "Invalid profile: %d floats", count);
        return;
    }
    profile_data = (float*)data;
    profile_pairs = count / 2;
    lap = 1;  // skip mapping lap, replay mode
    ESP_LOGI(TAG, "Profile loaded: %d pairs (gyro_ff + speed)", profile_pairs);
}

// Spatial sampling: record at ~5mm intervals using encoder odometry
void mapper_sample(void) {
    if (map_count >= MAP_MAX_SAMPLES) return;

    float l_spd = encoder_get_speed(0);
    float r_spd = encoder_get_speed(1);
    float avg_spd = (l_spd + r_spd) * 0.5f;
    total_dist += avg_spd / (float)MAP_SAMPLE_RATE_HZ;

    // Spatial decimation: only record every MAP_SPACING_MM
    if (total_dist - last_sample_dist < (MAP_SPACING_MM / 1000.0f)) return;
    last_sample_dist = total_dist;

    float line_pos = grayscale_get_line_position();
    float gyro_z = imu_get_gyro_z();  // direct measurement, no κ computation

    map[map_count].distance_m = total_dist;
    map[map_count].gyro_z     = gyro_z;
    map[map_count].line_pos   = line_pos;
    map[map_count].speed_mps  = avg_spd;
    map_count++;
}

// Predict: look up gyro_ff and target_speed from profile or map
void mapper_predict(void) {
    if (map_count < 2) { next_cmd.active = false; return; }

    if (profile_data && profile_pairs > 0) {
        // ── Profile replay mode (lap 2+) ──────────────────────────────────
        if (lap_reference_length < 0.001f) { next_cmd.active = false; return; }

        // Normalize current position to [0, 1] over reference lap length
        float ratio = total_dist / lap_reference_length;
        // Allow exceeding reference length (safety margin)
        if (ratio > 1.05f) { next_cmd.active = false; return; }
        if (ratio < 0.0f)  ratio = 0.0f;

        float fidx = ratio * (float)(profile_pairs - 1);
        int idx = (int)fidx;
        if (idx >= profile_pairs - 1) idx = profile_pairs - 2;
        if (idx < 0) idx = 0;

        // Linear interpolate between adjacent profile entries
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

    // ── Mapping lap mode (lap 0): simple curvature-based speed limit ──────
    // No feedforward during mapping — just limit speed based on curvature
    float lookahead = total_dist + 0.1f;  // 10 cm look-ahead
    int idx = 0; float best_d = 1e9f;
    for (int i = 0; i < map_count; i++) {
        float d = fabsf(map[i].distance_m - lookahead);
        if (d < best_d) { best_d = d; idx = i; }
    }
    // Estimate curvature from recorded gyro
    float gyro_abs = fabsf(map[idx].gyro_z);
    float omega_rad = gyro_abs * 0.0174533f;  // °/s → rad/s
    float v_est = (fabsf(map[idx].speed_mps) > 0.05f) ? fabsf(map[idx].speed_mps) : MAX_SPEED_MPS;
    float k = omega_rad / v_est;
    if (k < 0.01f) k = 0.01f;
    float v_max = sqrtf(3.924f / k);  // μg/κ
    if (v_max > MAX_SPEED_MPS) v_max = MAX_SPEED_MPS;

    next_cmd.active = true;
    next_cmd.target_speed = v_max;
    next_cmd.feedforward_omega = 0;  // no FF on mapping lap
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
