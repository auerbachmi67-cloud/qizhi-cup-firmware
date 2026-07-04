#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"

static const char *TAG = "IMU";
#define I2C_PORT I2C_NUM_0
#define IMU_ADDR 0x68
static float gyro_z_off = 0;
static bool imu_present = false;
#define GYRO_SENSITIVITY 131.0f  // ±250°/s default MPU6050, LSB/(°/s)

static void imu_write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_write_byte(c, val, true);
    i2c_master_stop(c);

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
        i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(10));
        xSemaphoreGive(i2c_mutex);
    }
    i2c_cmd_link_delete(c);
}

static uint8_t imu_read_byte(uint8_t reg) {
    uint8_t data = 0;
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(c, &data, I2C_MASTER_NACK);
    i2c_master_stop(c);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(c);

    return (ret == ESP_OK) ? data : 0x00;
}

static int16_t imu_read_int16(uint8_t reg) {
    static int16_t last_val[256] = {0};

    if (!imu_present) return 0;

    // 零延时拿锁，拿不到直接返回缓存值（OLED 在刷屏时不影响 1kHz 循环）
    if (xSemaphoreTake(i2c_mutex, 0) != pdTRUE) {
        return last_val[reg];
    }

    uint8_t b[2];
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);

    // Repeated Start — 一次原子事务
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(c, &b[0], I2C_MASTER_ACK);
    i2c_master_read_byte(c, &b[1], I2C_MASTER_NACK);
    i2c_master_stop(c);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(c);

    if (ret == ESP_OK) {
        last_val[reg] = (int16_t)((b[0] << 8) | b[1]);
    }

    xSemaphoreGive(i2c_mutex);
    return last_val[reg];
}

void imu_init(void) {
    uint8_t who_am_i = imu_read_byte(0x75);

    if (who_am_i == 0x68) {
        imu_present = true;
        imu_write_reg(0x6B, 0x00);  // wake up MPU6050
        ESP_LOGI(TAG, "MPU6050 init OK (WHO_AM_I=0x%02X)", who_am_i);
    } else {
        imu_present = false;
        ESP_LOGE(TAG, "MPU6050 NOT DETECTED (WHO_AM_I=0x%02X)! IMU bypassed.", who_am_i);
    }
}

void imu_calibrate(void) {
    if (!imu_present) return;

    gyro_z_off = 0;
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += imu_read_int16(0x47) / GYRO_SENSITIVITY;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_z_off = sum / 100.0f;
    ESP_LOGI(TAG, "Gyro Z bias: %.2f deg/s", gyro_z_off);
}

float imu_get_gyro_z(void) {
    return (imu_read_int16(0x47) / GYRO_SENSITIVITY) - gyro_z_off;
}
float imu_get_accel_z(void) {
    return imu_read_int16(0x3F) / 16384.0f;
}
