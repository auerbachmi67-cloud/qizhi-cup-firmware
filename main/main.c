#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "speed_control.h"
#include "line_control.h"
#include "ringbuf.h"
#include "mapper.h"
#include "wifi.h"
#include "oled.h"

static const char *TAG = "MAIN";
static SemaphoreHandle_t timer_sem = NULL;

static bool IRAM_ATTR timer_cb(gptimer_handle_t t, const gptimer_alarm_event_data_t *d, void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(timer_sem, &woken);
    return (woken == pdTRUE);
}

// ── Serial map export (short keys: d, p, k, v) ─────────────────────────
static void export_map_json(void) {
    int n = mapper_get_count();
    if (n == 0) { printf("{\"error\":\"no data\"}\n"); return; }
    const map_sample_t *s = mapper_get_samples();
    printf("{\"lap\":%d,\"total_dist_m\":%.3f,\"n\":%d,\"samples\":[",
           mapper_get_lap(), mapper_get_total_distance(), n);
    for (int i = 0; i < n; i++) {
        if (i > 0) printf(",");
        printf("{\"d\":%.3f,\"p\":%.2f,\"k\":%.4f,\"v\":%.3f}",
               s[i].distance_m, s[i].line_pos, s[i].curvature, s[i].speed_mps);
    }
    printf("]}\n");
}

// ── Serial command handler ───────────────────────────────────────────────
static void serial_task(void *arg) {
    uart_config_t uc = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uc);
    uart_set_pin(UART_NUM_0, 43, 44, -1, -1);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // Heap buffer: pload hex data can be > 128 bytes (100 floats = 800 hex chars)
    const int rx_size = 2048;
    char *buf = malloc(rx_size);
    if (!buf) { vTaskDelete(NULL); return; }

    while (1) {
        // Read as much as the FIFO currently holds
        int total = uart_read_bytes(UART_NUM_0, (uint8_t*)buf, rx_size - 1, pdMS_TO_TICKS(100));
        if (total <= 0) continue;
        buf[total] = 0;

        // Find the last complete line (trim trailing partial / noise)
        for (int i = 0; i < total; i++) {
            if (buf[i] == '\n' || buf[i] == '\r') {
                total = i;
                buf[i] = 0;
                break;
            }
        }
        if (total == 0) continue;

        if (strcmp(buf, "map") == 0) {
            export_map_json();
        } else if (strcmp(buf, "reset") == 0) {
            mapper_reset();
            printf("{\"ok\":\"mapper reset\"}\n");
        } else if (strcmp(buf, "lap") == 0) {
            printf("{\"lap\":%d,\"dist\":%.3f,\"samples\":%d}\n",
                   mapper_get_lap(), mapper_get_total_distance(), mapper_get_count());
        } else if (strncmp(buf, "pload:", 6) == 0) {
            // load speed profile: pload:N:hexdata
            char *p = buf + 6;
            int n = atoi(p);
            p = strchr(p, ':'); if (!p) { printf("{\"error\":\"bad format\"}\n"); continue; }
            p++;
            int hexlen = (int)strlen(p);
            int want = n * 4 * 2; // n floats * 4 bytes * 2 hex chars
            if (hexlen < want) { printf("{\"error\":\"hex too short\"}\n"); continue; }
            int flen = n * 4;
            uint8_t *raw = (uint8_t*)malloc(flen);
            if (!raw) { printf("{\"error\":\"oom\"}\n"); continue; }
            for (int i = 0; i < flen; i++) {
                char h[3] = {p[i*2], p[i*2+1], 0};
                raw[i] = (uint8_t)strtol(h, NULL, 16);
            }
            float *profile = (float*)raw;
            mapper_load_profile(profile, n);
            printf("{\"ok\":\"loaded %d points\",\"n\":%d}\n", n, n);
        } else {
            // Unknown command — echo back
            printf("{\"unknown\":\"%s\"}\n", buf);
        }
        uart_flush(UART_NUM_0);
    }
}

// ── Planner ──────────────────────────────────────────────────────────────
static void planner_task(void *arg) {
    ESP_LOGI(TAG, "Planner on Core 0");
    mapper_init(); wifi_init(); oled_init(); oled_show_splash();
    while (1) {
        mapper_sample(); mapper_predict();
        ringbuf_push(mapper_get_command());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ── Control ──────────────────────────────────────────────────────────────
static void control_task(void *arg) {
    ESP_LOGI(TAG, "Control on Core 1");

    ESP_LOGI(TAG, "Calibrating sensors...");
    grayscale_init();
    grayscale_calibrate();
    imu_init();
    imu_calibrate();
    encoder_init();
    encoder_reset(0); encoder_reset(1);
    motor_init();
    speed_control_init();
    line_control_init();

    timer_sem = xSemaphoreCreateBinary();
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    gptimer_new_timer(&tcfg, &gptimer);
    gptimer_alarm_config_t acfg = {
        .alarm_count = 1000, .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true },
    };
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_cb };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    gptimer_set_alarm_action(gptimer, &acfg);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    float target_speed = MAX_SPEED_MPS;
    int tick = 0;
    ESP_LOGI(TAG, "1kHz loop running");

    while (1) {
        xSemaphoreTake(timer_sem, portMAX_DELAY);

        float line_pos = grayscale_get_line_position();
        float left_spd  = encoder_get_speed(0);
        float right_spd = encoder_get_speed(1);
        float avg_spd   = (left_spd + right_spd) * 0.5f;

        control_cmd_t ff = ringbuf_pop();
        if (ff.active) target_speed = ff.target_speed;

        float base_throttle = speed_control_update(target_speed, avg_spd);
        float diff = line_control_update(line_pos);

        // Anti-windup speed compensation: if diff saturates, derate target
        float left  = base_throttle + diff;
        float right = base_throttle - diff;
        float sat = 0.0f;
        if (left  >  1.0f) { sat = left  - 1.0f; left  = 1.0f; }
        if (left  < -1.0f) { sat = 1.0f + left;   left  = -1.0f; }
        if (right >  1.0f) { sat = right - 1.0f;  right = 1.0f; }
        if (right < -1.0f) { sat = 1.0f + right;  right = -1.0f; }
        // Conditional integral hold: freeze speed_pid integral when saturated
        if (sat > 0.0f) speed_hold_integral();
        else            speed_release_integral();

        motor_set_speed(0, left);
        motor_set_speed(1, right);

        if (++tick >= 50) { tick = 0;
            wifi_send_telemetry(line_pos, left_spd, right_spd, target_speed,
                                imu_get_gyro_z(), left, right);
        }
    }
}

// ── Entry ────────────────────────────────────────────────────────────────
void app_main(void) {
    ESP_LOGI(TAG, "[Qizhi-Cup] ESP32-S3 + TB6612FNG v0.8");
    xTaskCreatePinnedToCore(serial_task,   "uart", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(planner_task,  "plan", 4096, NULL, configMAX_PRIORITIES-2, NULL, 0);
    xTaskCreatePinnedToCore(control_task,  "ctrl", 4096, NULL, configMAX_PRIORITIES-1, NULL, 1);
}
