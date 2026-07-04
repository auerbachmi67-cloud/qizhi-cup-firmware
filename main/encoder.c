#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ENC";

static float current_speed[2] = {0.0f, 0.0f};
static int32_t enc_last[2] = {0, 0};
static pcnt_unit_handle_t pcnt_unit[2] = {NULL, NULL};
static uint32_t last_time_us = 0;

static void pcnt_init_unit(int ch, int pinA, int pinB) {
    pcnt_unit_config_t unit_cfg = { .low_limit = -32767, .high_limit = 32767 };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, &pcnt_unit[ch]);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "encoder ch%d: pcnt_new_unit failed", ch);
        return;
    }
    pcnt_glitch_filter_config_t filt_cfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(pcnt_unit[ch], &filt_cfg);
    pcnt_chan_config_t chan_cfg_a = { .edge_gpio_num = pinA, .level_gpio_num = pinB };
    pcnt_chan_config_t chan_cfg_b = { .edge_gpio_num = pinB, .level_gpio_num = pinA };
    pcnt_channel_handle_t ch_a = NULL, ch_b = NULL;
    pcnt_new_channel(pcnt_unit[ch], &chan_cfg_a, &ch_a);
    pcnt_new_channel(pcnt_unit[ch], &chan_cfg_b, &ch_b);
    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(pcnt_unit[ch]);
    pcnt_unit_clear_count(pcnt_unit[ch]);
    pcnt_unit_start(pcnt_unit[ch]);
}

void encoder_init(void) {
    pcnt_init_unit(0, 17, 18);
    pcnt_init_unit(1, 10, 11);
    last_time_us = (uint32_t)esp_timer_get_time();
    ESP_LOGI(TAG, "Encoder initialized");
}

void encoder_update(void) {
    uint32_t now = (uint32_t)esp_timer_get_time();
    float dt = (now - last_time_us) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last_time_us = now;

    for (int ch = 0; ch < 2; ch++) {
        if (!pcnt_unit[ch]) continue;
        int count = 0;
        pcnt_unit_get_count(pcnt_unit[ch], &count);
        int d = count - enc_last[ch];
        enc_last[ch] = count;
        current_speed[ch] = (d * 0.000172787f) / dt;
    }
}

float encoder_get_speed(int ch) {
    if (ch < 0 || ch > 1) return 0.0f;
    return current_speed[ch];
}

int32_t encoder_get_count(int ch) {
    if (ch < 0 || ch > 1 || !pcnt_unit[ch]) return 0;
    int count = 0;
    pcnt_unit_get_count(pcnt_unit[ch], &count);
    return count;
}

void encoder_reset(int ch) {
    if (ch >= 0 && ch <= 1 && pcnt_unit[ch]) {
        pcnt_unit_clear_count(pcnt_unit[ch]);
        enc_last[ch] = 0;
    }
}
