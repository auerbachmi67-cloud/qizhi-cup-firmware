#include "oled.h"
#include "driver/i2c.h"
#include "esp_log.h"
static const char *TAG = "OLED";
#define OLED_ADDR 0x3C

static void oled_write_cmd(uint8_t c) {
    i2c_cmd_handle_t x = i2c_cmd_link_create();
    i2c_master_start(x);
    i2c_master_write_byte(x, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(x, 0x00, true);
    i2c_master_write_byte(x, c, true);
    i2c_master_stop(x);
    i2c_master_cmd_begin(0, x, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(x);
}

void oled_init(void) {
    vTaskDelay(pdMS_TO_TICKS(10));
    oled_write_cmd(0xAE); oled_write_cmd(0x20); oled_write_cmd(0x00);
    oled_write_cmd(0xB0); oled_write_cmd(0xC8); oled_write_cmd(0x00);
    oled_write_cmd(0x10); oled_write_cmd(0x40); oled_write_cmd(0x81);
    oled_write_cmd(0xFF); oled_write_cmd(0xA1); oled_write_cmd(0xA6);
    oled_write_cmd(0xA8); oled_write_cmd(0x3F); oled_write_cmd(0xA4);
    oled_write_cmd(0xD3); oled_write_cmd(0x00); oled_write_cmd(0xD5);
    oled_write_cmd(0xF0); oled_write_cmd(0xD9); oled_write_cmd(0x22);
    oled_write_cmd(0xDA); oled_write_cmd(0x12); oled_write_cmd(0xDB);
    oled_write_cmd(0x20); oled_write_cmd(0x8D); oled_write_cmd(0x14);
    oled_write_cmd(0xAF);
    ESP_LOGI(TAG, "OLED initialized");
}

void oled_show_splash(void) { ESP_LOGI(TAG, "Splash screen"); }
void oled_printf(const char *fmt, ...) { (void)fmt; }
