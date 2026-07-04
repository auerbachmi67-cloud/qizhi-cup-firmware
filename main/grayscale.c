#include "grayscale.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GS";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static float last_line_pos = 0.0f;
static bool calibrated = true;

void grayscale_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < 8; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, i, &chan_cfg));
    }
    ESP_LOGI(TAG, "Grayscale initialized (ADC1, 8 channels)");
}

void grayscale_read(uint16_t *out) {
    int raw;
    for (int i = 0; i < 8; i++) {
        adc_oneshot_read(adc_handle, i, &raw);
        out[i] = raw;
    }
}

void grayscale_calibrate(void) {
    ESP_LOGI(TAG, "Calibration not required for oneshot raw weighted mode");
}

float grayscale_get_line_position(void) {
    if (!adc_handle) return last_line_pos;
    int raw[8];
    float ws = 0, t = 0;
    for (int i = 0; i < 8; i++) {
        adc_oneshot_read(adc_handle, i, &raw[i]);
        ws += (float)i * raw[i];
        t  += raw[i];
    }
    if (t < 10.0f) return last_line_pos;
    last_line_pos = (ws / t) - 3.5f;
    return last_line_pos;
}

void grayscale_set_threshold(float t) { (void)t; }
bool grayscale_is_calibrated(void) { return calibrated; }
