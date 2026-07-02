#ifndef __FONT_H
#define __FONT_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- ASCII 字体 ---------- */

typedef struct {
  uint8_t h;
  uint8_t w;
  uint8_t *chars;
} ASCIIFont;

extern const ASCIIFont afont8x6;
extern const ASCIIFont afont12x6;
extern const ASCIIFont afont16x8;
extern const ASCIIFont afont24x12;

/* ---------- 中文字体 ---------- */

/**
 * 字库前 4 字节存储 UTF-8 编码，剩余字节存储字模数据
 * 可使用波特律动 LED 取模助手生成 (https://led.baud-dance.com)
 */
typedef struct {
  uint8_t h;              // 字体高度
  uint8_t w;              // 字体宽度
  const uint8_t *chars;   // 字库数据
  uint8_t len;            // 字库字符数（超 256 请改为 uint16_t）
  const ASCIIFont *ascii; // 缺省 ASCII 字体
} Font;

extern const Font font16x16;
extern const Font font15x20;

/* ---------- 图片 ---------- */

typedef struct {
  uint8_t w;           // 宽度
  uint8_t h;           // 高度
  const uint8_t *data; // 数据
} Image;

extern const Image bilibiliImg;
extern const Image m0Img;
extern const Image m1Img;
extern const Image m2Img;
extern const Image m3Img;
extern const Image m4Img;
extern const Image m5Img;

#ifdef __cplusplus
}
#endif

#endif // __FONT_H
