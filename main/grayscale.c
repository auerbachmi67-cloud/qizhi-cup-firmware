#include "grayscale.h"
#include "driver/adc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "GS";

static const adc1_channel_t CH[8] = {
    ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3,
    ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7,
};

static uint16_t raw[8];
static uint16_t calib_min[8], calib_max[8];
static bool calibrated = false;
static float last_position = 0.0f;
static int lost_count = 0;

void grayscale_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    for (int i = 0; i < 8; i++) {
        adc1_config_channel_atten(CH[i], ADC_ATTEN_DB_11);
        calib_min[i] = 4095; calib_max[i] = 0;
    }
    ESP_LOGI(TAG, "8ch grayscale ADC1 init OK");
}

void grayscale_read(uint16_t *out) {
    for (int i = 0; i < 8; i++)
        out[i] = adc1_get_raw(CH[i]);
}

void grayscale_calibrate(void) {
    ESP_LOGI(TAG, "Calibrating... move sensor over line & background");
    for (int n = 0; n < 200; n++) {
        grayscale_read(raw);
        for (int i = 0; i < 8; i++) {
            if (raw[i] < calib_min[i]) calib_min[i] = raw[i];
            if (raw[i] > calib_max[i]) calib_max[i] = raw[i];
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    calibrated = true;
    ESP_LOGI(TAG, "Calibration done");
    for (int i = 0; i < 8; i++)
        ESP_LOGI(TAG, "  ch%d: %d-%d", i, calib_min[i], calib_max[i]);
}

float grayscale_get_line_position(void) {
    grayscale_read(raw);

    // Normalize 0..1 using calibration
    float norm[8];
    int active = 0;
    for (int i = 0; i < 8; i++) {
        if (calibrated) {
            int range = (int)calib_max[i] - (int)calib_min[i];
            if (range < 10) range = 10;
            norm[i] = 1.0f - ((float)(raw[i] - calib_min[i]) / (float)range);
            if (norm[i] < 0.0f) norm[i] = 0.0f;
            if (norm[i] > 1.0f) norm[i] = 1.0f;
        } else {
            norm[i] = 1.0f - ((float)raw[i] / 4095.0f);
        }
        // Line = dark = low ADC value → high norm value
        if (norm[i] > 0.5f) active++;
    }

    // Lost line recovery: if < 2 sensors see line, use last position
    if (active < 2) {
        lost_count++;
        if (lost_count > 20) {
            // Extended loss — search in last known direction
            return (last_position > 0) ? 5.0f : -5.0f;
        }
        return last_position;
    }
    lost_count = 0;

    // Weighted center of gravity, index 0-7 mapped to -3.5..+3.5
    float sum = 0.0f, weighted = 0.0f;
    for (int i = 0; i < 8; i++) {
        sum += norm[i];
        weighted += norm[i] * (float)(i);
    }
    if (sum < 0.01f) return last_position;

    last_position = weighted / sum - 3.5f;
    return last_position;
}

void grayscale_set_threshold(float t) { (void)t; }
bool grayscale_is_calibrated(void) { return calibrated; }
