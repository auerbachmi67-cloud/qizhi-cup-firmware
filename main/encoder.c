#include "encoder.h"
#include "esp_log.h"

static const char *TAG = "ENC";

void encoder_init(void) {
    ESP_LOGW(TAG, "encoder skipped (gpio pin conflict with other peripherals)");
}

int32_t encoder_get_count(int ch) {
    (void)ch;
    return 0;
}

float encoder_get_speed(int ch) {
    (void)ch;
    return 0.0f;
}

void encoder_reset(int ch) {
    (void)ch;
}


int32_t encoder_get_count(int ch) {
    (void)ch;
    return 0;
}

float encoder_get_speed(int ch) {
    (void)ch;
    return 0.0f;
}

void encoder_reset(int ch) {
    (void)ch;
}
