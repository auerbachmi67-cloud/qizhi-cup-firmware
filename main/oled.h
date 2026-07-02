/**
 * @file oled.h
 * @brief OLED驱动(SSD1306) - 提取自 STM32F103 工程，已清理依赖
 *
 * 主要修改:
 *   1. 去掉 #include "main.h" 依赖，改用 <stdint.h> <string.h>
 *   2. 提取 I2C 发送接口为可配置回调（OLED_SetSendFunc / 宏）
 *   3. 保留所有绘图/文字 API 不变
 */
#ifndef __OLED_H__
#define __OLED_H__

#include "font.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 移植接口（二选一） ========== */

/**
 * @brief I2C 发送函数指针类型
 * @param data 数据指针
 * @param len  数据长度
 *
 * 用户在初始化前调用 OLED_SetSendFunc() 注册底层发送函数。
 * 若不想用函数指针，直接修改 oled.c 中的 OLED_Send() 函数体。
 */
typedef void (*OLED_SendFunc_t)(uint8_t *data, uint16_t len);
void OLED_SetSendFunc(OLED_SendFunc_t func);

/* ========== 颜色模式 ========== */

typedef enum {
  OLED_COLOR_NORMAL = 0, // 正常模式 黑底白字
  OLED_COLOR_REVERSED    // 反色模式 白底黑字
} OLED_ColorMode;

/* ========== 初始化 & 控制 ========== */

void OLED_Init();
void OLED_DisPlay_On();
void OLED_DisPlay_Off();
void OLED_SetColorMode(OLED_ColorMode mode);

/* ========== 显存操作 ========== */

void OLED_NewFrame();
void OLED_ShowFrame();
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color);

/* ========== 图形绘制 ========== */

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color);
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color);
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color);

/* ========== 文字绘制 ========== */

void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color);

/* ========== 底层字节操作（通常不需要直接调用） ========== */

void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color);
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color);
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color);
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color);
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color);

/* ========== 兼容包装（项目原有小写命名） ========== */

void oled_init(void);
void oled_show_splash(void);

/**
 * @brief 启动 OLED 图片轮播 + 队名显示任务
 *
 * 在 Core 0 上创建一个后台任务，按 300ms 间隔循环显示 6 张图片，
 * 底部固定显示 "动次嗒次队"。任务内部每帧都会 OLED_NewFrame() 清屏
 * 再 OLED_ShowFrame() 刷新，解决只显示不刷新的问题。
 */
void oled_start_carousel_task(void);

#ifdef __cplusplus
}
#endif

#endif // __OLED_H__
