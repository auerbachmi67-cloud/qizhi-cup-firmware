#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "IMU";
#define I2C_PORT I2C_NUM_0
#define IMU_ADDR 0x68
static float gyro_z_off = 0;
#define GYRO_SENSITIVITY 131.0f  // ±250°/s default MPU6050, LSB/(°/s)

static void imu_write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_write_byte(c, val, true);
    i2c_master_stop(c);
    i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
}

static int16_t imu_read_int16(uint8_t reg) {
    uint8_t b[2];
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_stop(c);
    i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (IMU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(c, &b[0], I2C_MASTER_ACK);
    i2c_master_read_byte(c, &b[1], I2C_MASTER_NACK);
    i2c_master_stop(c);
    i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return (int16_t)((b[0] << 8) | b[1]);
}

void imu_init(void) {
    // I2C 由 main.c 的 i2c_master_init() 统一初始化，此处只配置 MPU6050
    imu_write_reg(0x6B, 0x00);
    ESP_LOGI(TAG, "MPU6050 init OK (I2C_NUM_0, addr=0x%02X)", IMU_ADDR);
}

void imu_calibrate(void) {
    gyro_z_off = 0;
    for (int i = 0; i < 100; i++) {
        gyro_z_off += imu_read_int16(0x47) / GYRO_SENSITIVITY;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_z_off /= 100.0f;
    ESP_LOGI(TAG, "Gyro Z bias: %.2f deg/s", gyro_z_off);
}

float imu_get_gyro_z(void) { return imu_read_int16(0x47) / GYRO_SENSITIVITY - gyro_z_off; }
float imu_get_accel_z(void) { return imu_read_int16(0x3F) / 16384.0f; }
