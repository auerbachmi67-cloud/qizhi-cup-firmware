#include "encoder.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "ENC";

// GPIO: left encoder on 17/18, right on 21/22 — avoids ADC1 & motor zones
static const int EA[2] = {17, 21};
static const int EB[2] = {18, 22};
static const pcnt_unit_t EU[2] = {PCNT_UNIT_0, PCNT_UNIT_1};
static int16_t lc[2] = {0, 0};

void encoder_init(void) {
    for (int i = 0; i < 2; i++) {
        pcnt_config_t cfg = {
            .pulse_gpio_num = EA[i], .ctrl_gpio_num = EB[i],
            .channel = PCNT_CHANNEL_0, .unit = EU[i],
            .pos_mode = PCNT_COUNT_INC, .neg_mode = PCNT_COUNT_DEC,
            .lctrl_mode = PCNT_MODE_REVERSE, .hctrl_mode = PCNT_MODE_KEEP,
            .counter_h_lim = 32767, .counter_l_lim = -32767,
        };
        pcnt_unit_config(&cfg);
        pcnt_counter_pause(EU[i]); pcnt_counter_clear(EU[i]); pcnt_counter_resume(EU[i]);
    }
    ESP_LOGI(TAG, "Encoders init OK");
}

int32_t encoder_get_count(int ch) {
    int16_t v;
    pcnt_get_counter_value(EU[ch], &v);
    return v;
}

float encoder_get_speed(int ch) {
    // Called at CONTROL_LOOP_HZ (1kHz).
    // scale = PI * diameter_mm / (pulses_per_rev * gear_ratio) = delta per pulse in meters
    float scale = 3.14159f * WHEEL_DIAMETER_MM / (float)(PULSES_PER_REV * ENCODER_GEAR_RATIO);
    int16_t v;
    pcnt_get_counter_value(EU[ch], &v);
    int16_t d = v - lc[ch];
    lc[ch] = v;
    return d * scale;
}

void encoder_reset(int ch) {
    pcnt_counter_pause(EU[ch]); pcnt_counter_clear(EU[ch]); lc[ch] = 0; pcnt_counter_resume(EU[ch]);
}
