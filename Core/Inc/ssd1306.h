/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ssd1306.h
  * @brief   SSD1306 OLED驱动 (128x64, I2C接口)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// OLED软件I2C引脚定义 (PC11=SCL, PC12=SDA)
#define OLED_SCL_PORT       GPIOC
#define OLED_SCL_PIN        GPIO_PIN_11
#define OLED_SDA_PORT       GPIOC
#define OLED_SDA_PIN        GPIO_PIN_12

#define OLED_SCL_HIGH()     HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW()      HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH()     HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()      HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define OLED_SDA_READ()     HAL_GPIO_ReadPin(OLED_SDA_PORT, OLED_SDA_PIN)

// OLED参数
#define SSD1306_I2C_ADDR    0x78    // I2C地址 (7位0x3C << 1 = 0x78)
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_PAGES       8       // 64/8 = 8页

// 初始化和基本操作
void SSD1306_Init(void);
void SSD1306_Fill(uint8_t data);
void SSD1306_UpdateScreen(void);
void SSD1306_Clear(void);

// 文字显示 (6x8字体)
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_Putc(char ch);
void SSD1306_Puts(const char *str);
void SSD1306_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */
