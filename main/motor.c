#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

static const int IN1[2] = {40, 13};
static const int IN2[2] = {14, 38};
static const int PWM[2] = {42, 41};
static const int STBY = 39;

static float prev_throttle[2] = {0, 0};
static uint32_t brake_countdown[2] = {0, 0};

void motor_init(void) {
    ledc_timer_config_t tmr = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = PWM[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (i == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0, .hpoint = 0,
        };
        ledc_channel_config(&ch);
        gpio_set_direction(IN1[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(IN2[i], GPIO_MODE_OUTPUT);
        gpio_set_level(IN1[i], 0);
        gpio_set_level(IN2[i], 0);
        prev_throttle[i] = 0;
        brake_countdown[i] = 0;
    }
    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY, 1);
    ESP_LOGI(TAG, "TB6612FNG init OK");
}

void motor_set_speed(int channel, float throttle) {
    if (channel < 0 || channel > 1) return;
    ledc_channel_t ch = (channel == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;

    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < -1.0f) throttle = -1.0f;
    float abs_thr = fabsf(throttle);

    bool was_running = (fabsf(prev_throttle[channel]) > 0.02f);
    bool stopping = (abs_thr < 0.015f);

    if (stopping && was_running && brake_countdown[channel] == 0) {
        brake_countdown[channel] = 50;
    }

    if (brake_countdown[channel] > 0) {
        brake_countdown[channel]--;
        gpio_set_level(IN1[channel], 0);
        gpio_set_level(IN2[channel], 0);
        float ramp = (float)brake_countdown[channel] / 50.0f;
        uint32_t duty = (uint32_t)(fabsf(prev_throttle[channel]) * ramp * 1023.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
        if (brake_countdown[channel] == 0) prev_throttle[channel] = 0;
        return;
    }

    if (throttle > 0.015f) {
        gpio_set_level(IN1[channel], 1);
        gpio_set_level(IN2[channel], 0);
    } else if (throttle < -0.015f) {
        gpio_set_level(IN1[channel], 0);
        gpio_set_level(IN2[channel], 1);
    } else {
        gpio_set_level(IN1[channel], 0);
        gpio_set_level(IN2[channel], 0);
    }

    uint32_t duty = (uint32_t)(abs_thr * 1023.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    prev_throttle[channel] = throttle;
}

void motor_brake(int channel) {
    if (channel < 0 || channel > 1) return;
    ledc_channel_t ch = (channel == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    gpio_set_level(IN1[channel], 0);
    gpio_set_level(IN2[channel], 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    prev_throttle[channel] = 0;
    brake_countdown[channel] = 0;
}

void motor_coast(void) {
    gpio_set_level(IN1[0], 0); gpio_set_level(IN2[0], 0);
    gpio_set_level(IN1[1], 0); gpio_set_level(IN2[1], 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    prev_throttle[0] = 0; prev_throttle[1] = 0;
    brake_countdown[0] = 0; brake_countdown[1] = 0;
}
