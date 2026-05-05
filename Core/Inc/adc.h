/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

// 电池监控参数
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_1   // 使用PA1读取电池电压
#define BATTERY_VOLTAGE_MIN     9.0f            // 最低电压9V（根据你的电池调整）
#define BATTERY_VOLTAGE_MAX     13.0f           // 满电电压13V（根据你的电池调整）
#define BATTERY_DIVIDER_RATIO   5.0f            // 分压比例5:1（标准电压检测模块）
#define ADC_REF_VOLTAGE         3.3f            // ADC参考电压
#define ADC_RESOLUTION          4095.0f         // 12位ADC分辨率
#define BATTERY_CORRECTION      1.0073f         // 校准系数: 12.35/12.26 ≈ 1.0073
#define ADC_OVERSAMPLE_COUNT    8               // 过采样次数，取平均值降低噪声

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

// 电池监控函数
uint16_t ADC_ReadChannel(uint32_t channel);
float Battery_ReadVoltage(void);
uint8_t Battery_GetPercentage(float voltage);
void Battery_PrintStatus(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

