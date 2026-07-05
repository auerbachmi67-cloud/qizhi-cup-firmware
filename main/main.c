#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "config.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "mapper.h"
#include "motor.h"
#include "pid.h"
#include "ringbuf.h"
#include "speed_control.h"
#include "line_control.h"
#include "wifi.h"
#include "oled.h"
#include "vbat.h"

static const char *TAG = "MAIN";

SemaphoreHandle_t i2c_mutex = NULL;

typedef enum {
    STATE_IDLE = 0,
    STATE_ARMED,
    STATE_RUNNING
} system_state_t;

static volatile system_state_t sys_state = STATE_IDLE;
static volatile uint32_t g_watchdog_last_ms = 0;
static volatile bool g_button_pressed = false;

// Telemetry sharing between cores
static volatile float cur_lp = 0, cur_ls = 0, cur_rs = 0, cur_ts = 0, cur_gz = 0, cur_lt = 0, cur_rt = 0, cur_ff = 0;

static char uart_rxbuf[20480];
extern bool wifi_ready;

// Sensor health flags
static bool g_imu_ok = false;
static bool g_enc_ok = false;

// ── I2C ────────────────────────────────────────────────────────────────────

void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num  = 16,
        .scl_io_num  = 15,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// ── Button ──────────────────────────────────────────────────────────────────

static void IRAM_ATTR start_button_isr(void *arg) {
    g_button_pressed = true;
}

void start_button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << START_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(START_BUTTON_GPIO, start_button_isr, NULL);
    ESP_LOGI(TAG, "Start button on GPIO %d", START_BUTTON_GPIO);
}

static void handle_button_press(void) {
    if (!g_button_pressed) return;
    g_button_pressed = false;

    static uint32_t last_press_tick = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - last_press_tick < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) return;
    last_press_tick = now;

    if (sys_state == STATE_ARMED) {
        sys_state = STATE_RUNNING;
        speed_reset();
        ringbuf_init();
        g_watchdog_last_ms = xTaskGetTickCount();
        ESP_LOGI(TAG, ">>> RUNNING (button)");
    }
}

bool is_system_running(void) {
    return (sys_state == STATE_RUNNING);
}

// ── Command ─────────────────────────────────────────────────────────────────

static uint8_t hex_to_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static int process_command(char *cmd, char *out, int out_max) {
    if (strcmp(cmd, "map") == 0) {
        int m_count = mapper_get_count();
        const map_sample_t* m_data = mapper_get_samples();
        char buf[256]; int len;
        len = snprintf(buf, sizeof(buf), "{\"lap\":%d,\"total_dist_m\":%.2f,\"n\":%d,\"samples\":[",
                      mapper_get_lap(), mapper_get_total_distance(), m_count);
        uart_write_bytes(UART_NUM_0, buf, len);
        for (int i = 0; i < m_count; i++) {
            len = snprintf(buf, sizeof(buf), "{\"d\":%.2f,\"g\":%.4f,\"p\":%.2f,\"v\":%.2f}%s",
                          m_data[i].distance_m, m_data[i].gyro_z,
                          m_data[i].line_pos, m_data[i].speed_mps,
                          (i == m_count - 1) ? "" : ",");
            uart_write_bytes(UART_NUM_0, buf, len);
        }
        uart_write_bytes(UART_NUM_0, "]}\n", 3);
        return snprintf(out, out_max, "{\"ok\":\"map\",\"n\":%d}\n", (int)m_count);
    }
    if (strcmp(cmd, "lap") == 0) {
        return snprintf(out, out_max, "{\"lap\":%d,\"dist\":%.2f,\"samples\":%d,\"speed\":%.2f,\"state\":%d}\n",
                        mapper_get_lap(), mapper_get_total_distance(), (int)mapper_get_count(), cur_ts, (int)sys_state);
    }
    if (strcmp(cmd, "reset") == 0) {
        mapper_reset();
        return snprintf(out, out_max, "{\"ok\":\"mapper reset\"}\n");
    }
    if (strcmp(cmd, "ping") == 0) {
        g_watchdog_last_ms = xTaskGetTickCount();
        return snprintf(out, out_max, "{\"ok\":\"pong\"}\n");
    }
    if (strcmp(cmd, "arm") == 0) {
        if (sys_state == STATE_RUNNING) return snprintf(out, out_max, "{\"err\":\"already running\"}\n");
        sys_state = STATE_ARMED;
        ESP_LOGI(TAG, "ARMED");
        return snprintf(out, out_max, "{\"ok\":\"armed\"}\n");
    }
    if (strcmp(cmd, "go") == 0) {
        if (sys_state == STATE_IDLE) sys_state = STATE_ARMED;
        sys_state = STATE_RUNNING;
        speed_reset();
        ringbuf_init();
        g_watchdog_last_ms = xTaskGetTickCount();
        ESP_LOGI(TAG, ">>> RUNNING (go)");
        return snprintf(out, out_max, "{\"ok\":\"running\"}\n");
    }
    if (strcmp(cmd, "stop") == 0) {
        if (sys_state == STATE_RUNNING) {
            motor_stop_ramp(cur_lt, cur_rt);
        } else {
            motor_coast();
        }
        sys_state = STATE_IDLE;
        ESP_LOGI(TAG, "STOP");
        return snprintf(out, out_max, "{\"ok\":\"stopped\"}\n");
    }
    if (strcmp(cmd, "estop") == 0) {
        sys_state = STATE_IDLE;
        motor_coast();
        ESP_LOGW(TAG, "ESTOP");
        return snprintf(out, out_max, "{\"ok\":\"estop\"}\n");
    }

    if (strncmp(cmd, "pload:", 6) == 0) {
        if (sys_state == STATE_RUNNING) return snprintf(out, out_max, "{\"err\":\"stop first\"}\n");
        int n = 0;
        char *p = cmd + 6;
        while (*p != ':' && *p != '\0') { n = n * 10 + (*p - '0'); p++; }
        if (*p == ':') p++;
        if (n > 4096) n = 4096;
        static float profile_buf[4096];
        for (int i = 0; i < n; i++) {
            uint8_t bytes[4];
            for (int j = 0; j < 4; j++) {
                char c1 = p[i * 8 + j * 2];
                char c2 = p[i * 8 + j * 2 + 1];
                bytes[j] = (hex_to_val(c1) << 4) | hex_to_val(c2);
            }
            memcpy(&profile_buf[i], bytes, 4);
        }
        mapper_load_profile(profile_buf, n);
        ESP_LOGI(TAG, "Profile loaded: %d floats", n);
        return snprintf(out, out_max, "{\"ok\":\"loaded\",\"n\":%d}\n", n);
    }
    return snprintf(out, out_max, "{\"err\":\"unknown: %s\"}\n", cmd);
}

// ── Tasks ───────────────────────────────────────────────────────────────────

void uart_task(void *arg) {
    int idx = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)(uart_rxbuf + idx), 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            if (uart_rxbuf[idx] == '\n' || uart_rxbuf[idx] == '\r') {
                uart_rxbuf[idx] = '\0';
                if (idx > 0) {
                    char out[512];
                    int rlen = process_command(uart_rxbuf, out, sizeof(out));
                    if (rlen > 0) uart_write_bytes(UART_NUM_0, out, rlen);
                }
                idx = 0;
            } else {
                idx++;
                if (idx >= sizeof(uart_rxbuf)) idx = 0;
            }
        } else if (len < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void wifi_cmd_task(void *arg) {
    while (!wifi_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(COMMAND_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    while (1) {
        char buf[256];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &from_len);
        if (n < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (n == 0) continue;
        buf[n] = '\0';
        while (n > 0 && (buf[n-1] == '\r' || buf[n-1] == '\n')) buf[--n] = '\0';
        if (n == 0) continue;

        char out[512];
        int rlen = process_command(buf, out, sizeof(out));
        if (rlen > 0) {
            sendto(sock, out, rlen, 0, (struct sockaddr *)&from, from_len);
        }
    }
}

static void control_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Control on Core 1 @ %d Hz", CONTROL_LOOP_HZ);

    motor_init();
    encoder_init();
    grayscale_init();
    speed_control_init();
    line_control_init();

    ESP_LOGI(TAG, "Sensor status: IMU=%d ENC=%d GRAY=%d",
             (int)g_imu_ok, (int)g_enc_ok, (int)grayscale_is_ok());

    TickType_t lw = xTaskGetTickCount();

    while(1) {
        encoder_update();
        float ls = encoder_get_speed(0);
        float rs = encoder_get_speed(1);
        float avg_spd = (ls + rs) * 0.5f;

        float lp = grayscale_get_line_position();
        float gz = imu_get_gyro_z();

        cur_ls = ls; cur_rs = rs; cur_lp = lp; cur_gz = gz;

        // Core 1 independent watchdog — motor stop if planner (Core 0) is dead
        uint32_t now_tick = xTaskGetTickCount();
        if (sys_state == STATE_RUNNING &&
            (now_tick - g_watchdog_last_ms) > pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "ESTOP (watchdog: %lu ms stale)",
                     (now_tick - g_watchdog_last_ms) * portTICK_PERIOD_MS);
            sys_state = STATE_IDLE;
            motor_coast();
        }

        // Stop ramp tick (if active)
        motor_ramp_tick();

        if (sys_state == STATE_RUNNING) {
            control_cmd_t cmd = ringbuf_pop();
            if (cmd.active && ringbuf_is_fresh(cmd)) {
                cur_ts = cmd.target_speed;
                cur_ff = cmd.feedforward_omega;
            }

            float base_throttle = speed_control_update(cur_ts, avg_spd);
            float diff_throttle = line_control_update_ff(lp, cur_ff, avg_spd);

            float lt = base_throttle - diff_throttle;
            float rt = base_throttle + diff_throttle;

            cur_lt = lt; cur_rt = rt;
            motor_set_speed(0, lt);
            motor_set_speed(1, rt);
        } else {
            if (!motor_ramp_active()) {
                motor_coast();
            }
            cur_lt = 0; cur_rt = 0; cur_ts = 0; cur_ff = 0;
            speed_reset();
        }

        vTaskDelayUntil(&lw, pdMS_TO_TICKS(1));
    }
}

static void planner_task(void *arg) {
    ESP_LOGI(TAG, "Planner on Core 0 @ %d Hz", MAP_SAMPLE_RATE_HZ);
    vTaskDelay(pdMS_TO_TICKS(50));

    int tick = 0;
    TickType_t lw = xTaskGetTickCount();

    while (1) {
        handle_button_press();

        if (sys_state == STATE_RUNNING) {
            float avg_spd = (cur_ls + cur_rs) * 0.5f;
            mapper_sample(avg_spd, cur_lp, cur_gz);
            mapper_predict();
            control_cmd_t cmd = mapper_get_command();
            ringbuf_push(cmd);
        }

        tick++;
        if (tick % 5 == 0) {
            wifi_send_telemetry(cur_lp, cur_ls, cur_rs, cur_ts, cur_gz, cur_ff, cur_lt, cur_rt, sys_state == STATE_RUNNING);
        }

        vTaskDelayUntil(&lw, pdMS_TO_TICKS(10));
    }
}

// ── Main ────────────────────────────────────────────────────────────────────

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "[Qizhi-Cup] ESP32-S3 v0.17 — UART Grayscale + VBAT Comp + C1 Watchdog");

    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "WiFi MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    i2c_mutex = xSemaphoreCreateMutex();

    i2c_master_init();
    oled_init();
    imu_init();
    imu_calibrate();
    oled_start_carousel_task();

    start_button_init();
    wifi_init();

    xTaskCreatePinnedToCore(planner_task,  "plan",    8192, NULL, tskIDLE_PRIORITY + 3, NULL, 0);
    xTaskCreatePinnedToCore(wifi_cmd_task, "wificmd", 4096, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
    xTaskCreatePinnedToCore(control_task,  "ctrl",    8192, NULL, tskIDLE_PRIORITY + 4, NULL, 1);
    xTaskCreatePinnedToCore(uart_task,     "uart",    4096, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
}
