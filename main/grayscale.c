#include "grayscale.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "GRAY";

#define UART_NUM         UART_NUM_1
#define UART_TX          2
#define UART_RX          1
#define UART_BAUD        115200
#define UART_BUF_SIZE    1024

#define GRAY_CHANNELS    8

static bool g_ok = false;
static float g_line_pos = 0.0f;
static uint8_t g_digital = 0x00;
static uint16_t g_analog[GRAY_CHANNELS] = {0};

static int uart_rx_byte(uint8_t *b, int timeout_ms) {
    return uart_read_bytes(UART_NUM, b, 1, pdMS_TO_TICKS(timeout_ms));
}

void grayscale_init(void) {
    uart_config_t uc = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uc));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_flush_input(UART_NUM);

    const char *start_cmd = "$0,1,1#";
    uart_write_bytes(UART_NUM, start_cmd, strlen(start_cmd));
    ESP_LOGI(TAG, "UART1 init OK (GPIO %d/%d @ %d), sent: %s", UART_TX, UART_RX, UART_BAUD, start_cmd);

    uint8_t buf[128];
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline) {
        int n = uart_read_bytes(UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            buf[n] = '\0';
            if (strchr((char *)buf, '$') && strchr((char *)buf, '#')) {
                g_ok = true;
                ESP_LOGI(TAG, "Module responding — got %d bytes", n);
                return;
            }
        }
    }
    ESP_LOGW(TAG, "No response from grayscale module (check power + level shifter)");
}

void grayscale_calibrate(void) {
    const char *cal_cmd = "$1,0,0#";
    uart_write_bytes(UART_NUM, cal_cmd, strlen(cal_cmd));
    ESP_LOGI(TAG, "Calibration mode entered — press KEY on module");
    vTaskDelay(pdMS_TO_TICKS(2000));
    const char *resume_cmd = "$0,1,1#";
    uart_write_bytes(UART_NUM, resume_cmd, strlen(resume_cmd));
}

static bool parse_one_frame(void) {
    uint8_t b;
    TickType_t hunt_start = xTaskGetTickCount();
    while (1) {
        int n = uart_rx_byte(&b, 5);
        if (n <= 0) return false;
        if (b == '$') break;
        if ((xTaskGetTickCount() - hunt_start) > pdMS_TO_TICKS(100)) return false;
    }

    char payload[80];
    int idx = 0;
    while (idx < (int)sizeof(payload) - 1) {
        if (uart_rx_byte(&b, 5) <= 0) return false;
        if (b == '#') break;
        payload[idx++] = (char)b;
    }
    payload[idx] = '\0';

    if (payload[0] == 'D' && payload[1] == ',') {
        uint8_t dig = 0;
        char *p = payload + 2;
        for (int i = 0; i < GRAY_CHANNELS; i++) {
            p = strchr(p, ':');
            if (!p) break;
            p++;
            int val = 0;
            while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            if (val > 0) dig |= (1 << i);
        }
        g_digital = dig;
        return true;
    }

    if (payload[0] == 'A' && payload[1] == ',') {
        char *p = payload + 2;
        for (int i = 0; i < GRAY_CHANNELS; i++) {
            p = strchr(p, ':');
            if (!p) break;
            p++;
            int val = 0;
            while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            g_analog[i] = (uint16_t)val;
        }
        float ws = 0, t = 0;
        for (int i = 0; i < GRAY_CHANNELS; i++) {
            ws += (float)i * g_analog[i];
            t  += g_analog[i];
        }
        if (t > 10.0f) {
            g_line_pos = (ws / t) - 3.5f;
        }
        return true;
    }

    return false;
}

float grayscale_get_line_position(void) {
    if (!g_ok) return 0.0f;
    while (parse_one_frame()) {}
    return g_line_pos;
}

uint8_t grayscale_get_digital(void) {
    return g_digital;
}

bool grayscale_is_ok(void) {
    return g_ok;
}
