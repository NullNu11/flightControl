/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
// 电机PWM参数 (1MHz时钟, 1tick=1us, 50Hz标准舵机/ESC频率)
#define MOTOR_PWM_FREQ       50      // PWM频率 50Hz (标准ESC频率)
#define MOTOR_PWM_PERIOD     20000   // 周期 20000us (50Hz, 20ms)
#define MOTOR_PULSE_MIN      1000    // 最小脉宽 1000us (1ms)
#define MOTOR_PULSE_MAX      2000    // 最大脉宽 2000us (2ms)
#define MOTOR_PULSE_ARM      1000    // 解锁脉宽 1000us

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
// 电机控制函数
void Motor_Init(void);                                    // 初始化电机PWM
void Motor_SetAll(uint16_t pulse1, uint16_t pulse2, uint16_t pulse3, uint16_t pulse4);  // 设置所有电机脉宽
void Motor_ARM(void);                                     // 电机解锁（最小油门）
void Motor_Stop(void);                                    // 电机停止
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

