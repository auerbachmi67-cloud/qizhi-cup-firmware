#include "vbat.h"
#include "motor.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VBAT";

#define VBAT_NOMINAL_MPS 1.0f
#define VBAT_COMP_MAX    1.3f
#define VBAT_COMP_MIN    0.7f

void vbat_calibrate(void) {
    encoder_reset(0);
    encoder_reset(1);

    motor_set_speed(0, 1.0f);
    motor_set_speed(1, 1.0f);
    vTaskDelay(pdMS_TO_TICKS(200));

    encoder_update();
    float l = encoder_get_speed(0);
    float r = encoder_get_speed(1);
    motor_coast();
    vTaskDelay(pdMS_TO_TICKS(100));

    float avg = (l + r) * 0.5f;
    float comp = 1.0f;
    if (avg > 0.3f) {
        comp = VBAT_NOMINAL_MPS / avg;
        if (comp > VBAT_COMP_MAX) comp = VBAT_COMP_MAX;
        if (comp < VBAT_COMP_MIN) comp = VBAT_COMP_MIN;
    } else {
        ESP_LOGW(TAG, "Wheels not spinning — keeping comp=1.0");
    }
    motor_set_vbat_comp(comp);
    ESP_LOGI(TAG, "L=%.2f R=%.2f m/s  comp=%.3f", (double)l, (double)r, (double)comp);
}
