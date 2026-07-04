/**
 * @file    oled.c
 * @brief   OLED驱动(SSD1306) - 波特律动keysking驱动 + ESP-IDF I2C 胶水
 *
 * 结构:
 *   1. ESP-IDF I2C 发送包装 (oled_i2c_send)
 *   2. 提取的通用驱动 (s_send_func, 显存, 绘图, 文字)
 *   3. 兼容包装 (oled_init, oled_show_splash 小写接口)
 */
#include "oled.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include <stdlib.h>

static const char *TAG = "OLED";

/* ====================== ESP-IDF I2C 发送包装 ====================== */

#define OLED_I2C_PORT    I2C_NUM_0
static uint8_t s_oled_addr_7bit = 0x3C;  // 默认地址，扫描后会被覆盖

static void oled_i2c_send(uint8_t *data, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_oled_addr_7bit << 1) | I2C_MASTER_WRITE, true);
    for (uint16_t i = 0; i < len; i++)
        i2c_master_write_byte(cmd, data[i], true);
    i2c_master_stop(cmd);

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
        esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, pdMS_TO_TICKS(10));
        xSemaphoreGive(i2c_mutex);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "I2C TX fail: %s", esp_err_to_name(ret));
    } else {
        i2c_cmd_link_delete(cmd);
    }
}

/* ====================== 器件参数 ====================== */

#define OLED_ADDR    0x78   // SSD1306 I2C 地址（7位左移1位，保留供参考）
#define OLED_PAGE    8      // 页数
#define OLED_ROW     (8 * OLED_PAGE) // 64 行
#define OLED_COLUMN  128     // 128 列

/* ====================== 显存 ====================== */

static uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

/* ====================== 底层 I2C 发送 ====================== */

static OLED_SendFunc_t s_send_func = NULL;

void OLED_SetSendFunc(OLED_SendFunc_t func)
{
    s_send_func = func;
}

static void OLED_Send(uint8_t *data, uint8_t len)
{
    if (s_send_func)
        s_send_func(data, len);
}

static void OLED_SendCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    OLED_Send(buf, 2);
}

/* ====================== 初始化 & 控制 ====================== */

void OLED_Init()
{
    OLED_SendCmd(0xAE); // display off

    OLED_SendCmd(0x20); OLED_SendCmd(0x10);
    OLED_SendCmd(0xB0);
    OLED_SendCmd(0xC8);
    OLED_SendCmd(0x00); OLED_SendCmd(0x10);
    OLED_SendCmd(0x40);
    OLED_SendCmd(0x81); OLED_SendCmd(0xDF);
    OLED_SendCmd(0xA1);
    OLED_SendCmd(0xA6);
    OLED_SendCmd(0xA8); OLED_SendCmd(0x3F);
    OLED_SendCmd(0xA4);
    OLED_SendCmd(0xD3); OLED_SendCmd(0x00);
    OLED_SendCmd(0xD5); OLED_SendCmd(0xF0);
    OLED_SendCmd(0xD9); OLED_SendCmd(0x22);
    OLED_SendCmd(0xDA); OLED_SendCmd(0x12);
    OLED_SendCmd(0xDB); OLED_SendCmd(0x20);
    OLED_SendCmd(0x8D); OLED_SendCmd(0x14);

    OLED_NewFrame();
    OLED_ShowFrame();

    OLED_SendCmd(0xAF); // display on
}

void OLED_DisPlay_On()
{
    OLED_SendCmd(0x8D); OLED_SendCmd(0x14);
    OLED_SendCmd(0xAF);
}

void OLED_DisPlay_Off()
{
    OLED_SendCmd(0x8D); OLED_SendCmd(0x10);
    OLED_SendCmd(0xAE);
}

void OLED_SetColorMode(OLED_ColorMode mode)
{
    OLED_SendCmd(mode == OLED_COLOR_NORMAL ? 0xA6 : 0xA7);
}

/* ====================== 显存操作 ====================== */

void OLED_NewFrame()
{
    // 只清空图片区域（前 6 页：0~47 行，128×6=768 字节）
    // 底部文字区域（第 6~7 页：48~63 行）不碰，静态文字不动
    memset(OLED_GRAM, 0, 128 * 6);
}

void OLED_ShowFrame()
{
    // 跑起来时不刷新，冻结屏幕，I2C 全归 IMU
    if (is_system_running()) {
        return;
    }

    uint8_t buf[OLED_COLUMN + 1];
    buf[0] = 0x40;

    for (uint8_t i = 0; i < OLED_PAGE; i++)
    {
        // 每页独立拿锁+释放，中间 taskYIELD 让出 CPU
        // ⚠️ 注意回调路径：OLED_SendCmd() → OLED_Send() → s_send_func = oled_i2c_send
        // oled_i2c_send 内部自己拿锁，所以这里不重复拿锁

        OLED_SendCmd(0xB0 + i);
        OLED_SendCmd(0x00);
        OLED_SendCmd(0x10);
        memcpy(buf + 1, OLED_GRAM[i], OLED_COLUMN);
        OLED_Send(buf, OLED_COLUMN + 1);

        taskYIELD();
    }
}

void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color)
{
    if (x >= OLED_COLUMN || y >= OLED_ROW)
        return;
    if (color == OLED_COLOR_NORMAL)
        OLED_GRAM[y / 8][x] |= (1 << (y % 8));
    else
        OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
}

void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data,
                       uint8_t start, uint8_t end, OLED_ColorMode color)
{
    if (page >= OLED_PAGE || column >= OLED_COLUMN)
        return;
    if (color)
        data = ~data;
    uint8_t temp = data | (0xFF << (end + 1)) | (0xFF >> (8 - start));
    OLED_GRAM[page][column] &= temp;
    temp = data & ~(0xFF << (end + 1)) & ~(0xFF >> (8 - start));
    OLED_GRAM[page][column] |= temp;
}

void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color)
{
    if (page >= OLED_PAGE || column >= OLED_COLUMN)
        return;
    OLED_GRAM[page][column] = color ? ~data : data;
}

void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data,
                       uint8_t len, OLED_ColorMode color)
{
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    if (bit + len > 8)
    {
        OLED_SetByte_Fine(page,     x, data << bit,        bit, 7, color);
        OLED_SetByte_Fine(page + 1, x, data >> (8 - bit),  0, len + bit - 1 - 8, color);
    }
    else
    {
        OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
    }
}

void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color)
{
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    if (bit)
        OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
}

void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data,
                   uint8_t w, uint8_t h, OLED_ColorMode color)
{
    uint8_t fullRow = h / 8;
    uint8_t partBit = h % 8;
    for (uint8_t i = 0; i < w; i++)
        for (uint8_t j = 0; j < fullRow; j++)
            OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
    if (partBit)
    {
        uint16_t fullNum = w * fullRow;
        for (uint8_t i = 0; i < w; i++)
            OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
    }
}

/* ====================== 图形绘制 ====================== */

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color)
{
    if (x1 == x2)
    {
        if (y1 > y2) { uint8_t t = y1; y1 = y2; y2 = t; }
        for (uint8_t y = y1; y <= y2; y++) OLED_SetPixel(x1, y, color);
    }
    else if (y1 == y2)
    {
        if (x1 > x2) { uint8_t t = x1; x1 = x2; x2 = t; }
        for (uint8_t x = x1; x <= x2; x++) OLED_SetPixel(x, y1, color);
    }
    else
    {
        // Bresenham
        int16_t dx = x2 - x1, dy = y2 - y1;
        int16_t ux = ((dx > 0) << 1) - 1;
        int16_t uy = ((dy > 0) << 1) - 1;
        int16_t x = x1, y = y1, eps = 0;
        dx = abs(dx); dy = abs(dy);
        if (dx > dy)
        {
            for (x = x1; x != x2; x += ux)
            {
                OLED_SetPixel(x, y, color);
                eps += dy;
                if ((eps << 1) >= dx) { y += uy; eps -= dx; }
            }
        }
        else
        {
            for (y = y1; y != y2; y += uy)
            {
                OLED_SetPixel(x, y, color);
                eps += dx;
                if ((eps << 1) >= dy) { x += ux; eps -= dy; }
            }
        }
    }
}

void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
    OLED_DrawLine(x, y, x + w, y, color);
    OLED_DrawLine(x, y + h, x + w, y + h, color);
    OLED_DrawLine(x, y, x, y + h, color);
    OLED_DrawLine(x + w, y, x + w, y + h, color);
}

void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
    for (uint8_t i = 0; i < h; i++)
        OLED_DrawLine(x, y + i, x + w, y + i, color);
}

void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                       uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
    OLED_DrawLine(x1, y1, x2, y2, color);
    OLED_DrawLine(x2, y2, x3, y3, color);
    OLED_DrawLine(x3, y3, x1, y1, color);
}

void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                             uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
    uint8_t a = 0, b = 0, y = 0, last = 0;
    if (y1 > y2) { a = y2; b = y1; } else { a = y1; b = y2; }
    y = a;
    for (; y <= b; y++)
    {
        if (y <= y3)
            OLED_DrawLine(x1 + (int32_t)(y - y1) * (x2 - x1) / (y2 - y1), y,
                          x1 + (int32_t)(y - y1) * (x3 - x1) / (y3 - y1), y, color);
        else { last = y - 1; break; }
    }
    for (; y <= b; y++)
        OLED_DrawLine(x2 + (int32_t)(y - y2) * (x3 - x2) / (y3 - y2), y,
                      x1 + (int32_t)(y - last) * (x3 - x1) / (y3 - last), y, color);
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);
    while (a <= b)
    {
        OLED_SetPixel(x - b, y - a, color); OLED_SetPixel(x + b, y - a, color);
        OLED_SetPixel(x - a, y + b, color); OLED_SetPixel(x - b, y - a, color);
        OLED_SetPixel(x - a, y - b, color); OLED_SetPixel(x + b, y + a, color);
        OLED_SetPixel(x + a, y - b, color); OLED_SetPixel(x + a, y + b, color);
        OLED_SetPixel(x - b, y + a, color);
        a++;
        if (di < 0) di += 4 * a + 6;
        else { di += 10 + 4 * (a - b); b--; }
        OLED_SetPixel(x + a, y + b, color);
    }
}

void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);
    while (a <= b)
    {
        for (int16_t i = x - b; i <= x + b; i++)
        { OLED_SetPixel(i, y + a, color); OLED_SetPixel(i, y - a, color); }
        for (int16_t i = x - a; i <= x + a; i++)
        { OLED_SetPixel(i, y + b, color); OLED_SetPixel(i, y - b, color); }
        a++;
        if (di < 0) di += 4 * a + 6;
        else { di += 10 + 4 * (a - b); b--; }
    }
}

/* 椭圆：中点算法，全整数 */
void OLED_DrawEllipse(uint8_t cx, uint8_t cy, uint8_t a, uint8_t b, OLED_ColorMode color)
{
    int16_t x = 0, y = b;
    int32_t a2 = (int32_t)a * a, b2 = (int32_t)b * b;
    int32_t d = b2 + a2 * (1 - 2 * b) / 4;
    while (a2 * y > b2 * x)
    {
        OLED_SetPixel(cx + x, cy + y, color); OLED_SetPixel(cx - x, cy + y, color);
        OLED_SetPixel(cx + x, cy - y, color); OLED_SetPixel(cx - x, cy - y, color);
        if (d < 0)
        {
            d += b2 * ((x << 1) + 3);
            x++;
        }
        else
        {
            d += b2 * ((x << 1) + 3) + a2 * (-(y << 1) + 2);
            x++; y--;
        }
    }
    d = b2 * (x + 1) * (x + 1) / 4 + a2 * (y - 1) * (y - 1) - a2 * b2;
    while (y > 0)
    {
        OLED_SetPixel(cx + x, cy + y, color); OLED_SetPixel(cx - x, cy + y, color);
        OLED_SetPixel(cx + x, cy - y, color); OLED_SetPixel(cx - x, cy - y, color);
        if (d < 0)
        {
            d += b2 * ((x << 1) + 2) + a2 * (-(y << 1) + 3);
            x++; y--;
        }
        else
        {
            d += a2 * (-(y << 1) + 3);
            y--;
        }
    }
}

void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color)
{
    OLED_SetBlock(x, y, img->data, img->w, img->h, color);
}

/* ====================== 文字绘制 ====================== */

void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch,
                         const ASCIIFont *font, OLED_ColorMode color)
{
    OLED_SetBlock(x, y,
        font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w),
        font->w, font->h, color);
}

void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str,
                           const ASCIIFont *font, OLED_ColorMode color)
{
    while (*str)
    {
        OLED_PrintASCIIChar(x, y, *str, font, color);
        x += font->w;
        str++;
    }
}

/* UTF-8 编码长度 */
static uint8_t _OLED_GetUTF8Len(char *s)
{
    if ((s[0] & 0x80) == 0x00) return 1;
    if ((s[0] & 0xE0) == 0xC0) return 2;
    if ((s[0] & 0xF0) == 0xE0) return 3;
    if ((s[0] & 0xF8) == 0xF0) return 4;
    return 0;
}

void OLED_PrintString(uint8_t x, uint8_t y, char *str,
                      const Font *font, OLED_ColorMode color)
{
    uint16_t i = 0;
    uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4;
    uint8_t found, utf8Len;
    uint8_t *head;

    while (str[i])
    {
        found = 0;
        utf8Len = _OLED_GetUTF8Len(str + i);
        if (utf8Len == 0) break;

        for (uint8_t j = 0; j < font->len; j++)
        {
            head = (uint8_t *)(font->chars) + (j * oneLen);
            if (memcmp(str + i, head, utf8Len) == 0)
            {
                OLED_SetBlock(x, y, head + 4, font->w, font->h, color);
                x += font->w;
                i += utf8Len;
                found = 1;
                break;
            }
        }
        if (!found)
        {
            if (utf8Len == 1)
                OLED_PrintASCIIChar(x, y, str[i], font->ascii, color);
            else
                OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
            x += font->ascii->w;
            i += utf8Len;
        }
    }
}

/* ====================== 兼容包装（项目原有小写接口） ====================== */

void oled_init(void)
{
    // I2C 总线扫描：检测 OLED 地址（0x3C 或 0x3D）
    uint8_t test_addrs[] = {0x3C, 0x3D};
    bool found = false;
    for (int i = 0; i < sizeof(test_addrs); i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (test_addrs[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = ESP_FAIL;
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, pdMS_TO_TICKS(20));
            xSemaphoreGive(i2c_mutex);
        }
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            s_oled_addr_7bit = test_addrs[i];
            ESP_LOGI(TAG, "OLED found at 0x%02X", test_addrs[i]);
            found = true;
            break;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "OLED not detected on I2C bus! Check wiring/power (SDA=16, SCL=15)");
    }

    OLED_SetSendFunc(oled_i2c_send);
    vTaskDelay(pdMS_TO_TICKS(20));
    OLED_Init();
    ESP_LOGI(TAG, "Initialized at 0x%02X", s_oled_addr_7bit);
}

void oled_show_splash(void)
{
    OLED_NewFrame();
    OLED_PrintString(20, 45, "动次嗒次队", &font15x20, OLED_COLOR_NORMAL);
    OLED_DrawImage(37, 0, &m0Img, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    ESP_LOGI(TAG, "Splash shown");
}

/* ====================== 图片轮播 + 队名显示任务 ====================== */

static void oled_carousel_task(void *arg)
{
    const Image *images[] = {
        &m0Img, &m1Img, &m2Img, &m3Img, &m4Img, &m5Img
    };
    const int img_count = sizeof(images) / sizeof(images[0]);
    int img_id = 0;

    ESP_LOGI(TAG, "Carousel task started, %d images", img_count);

    // 底部文字只画一次，不参与轮播
    OLED_PrintString(25, 45, "动次嗒次队", &font15x20, OLED_COLOR_NORMAL);

    while (1) {
        // 跑起来时不刷新
        if (!is_system_running()) {
            OLED_NewFrame();  // 只清图片区，保护底部文字

            const Image *img = images[img_id];
            uint8_t x = (img_id == 1) ? 39 : 37;
            OLED_DrawImage(x, 0, img, OLED_COLOR_NORMAL);

            OLED_ShowFrame();

            img_id = (img_id + 1) % img_count;
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void oled_start_carousel_task(void)
{
    xTaskCreatePinnedToCore(oled_carousel_task, "oled_carousel", 3072, NULL,
                            tskIDLE_PRIORITY + 1, NULL, 0);
}
