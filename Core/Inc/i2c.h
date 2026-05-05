/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>

// MPU6050寄存器地址
#define MPU6050_ADDR            0xD0  // I2C地址 (7位: 0x68 << 1 = 0xD0)
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_WHO_AM_I        0x75

// 互补滤波器参数（旧版本 - 已注释）
// #define ALPHA                  0.96f   // 陀螺仪权重（降低以减少长期漂移）
// #define DT                     0.02f   // 采样时间(秒)

// 卡尔曼滤波器参数（新版本 - 当前使用）
// dt通过实际测量获得，不再使用固定值
#define KALMAN_Q_ANGLE         0.3f    // 过程噪声协方差 - 角度（增大以加快响应）
#define KALMAN_Q_BIAS          0.01f   // 过程噪声协方差 - 陀螺仪偏移
#define KALMAN_R_MEASURE       0.01f   // 测量噪声协方差（减小以信任加速度计更多，减少延迟）

// 角度平滑和死区参数
#define ANGLE_DEADZONE         0.1f    // 死区范围（度）：小于此值视为0
#define SMOOTH_FACTOR          1.0f    // 平滑系数：1.0=不使用平滑（卡尔曼已有平滑效果）

// 角度结构体
typedef struct {
    float pitch;   // 俯仰角
    float roll;    // 翻滚角
} Angle_t;

// 卡尔曼滤波器结构体（一维卡尔曼滤波）
typedef struct {
    float angle;    // 估计角度
    float bias;     // 估计陀螺仪偏移
    float P[2][2];  // 误差协方差矩阵 2x2
} Kalman_t;

// 校准结构体
typedef struct {
    int16_t gyro_x_offset;  // 陀螺仪X轴零偏
    int16_t gyro_y_offset;  // 陀螺仪Y轴零偏
    int16_t gyro_z_offset;  // 陀螺仪Z轴零偏
    int16_t accel_x_offset; // 加速度计X轴零偏
    int16_t accel_y_offset; // 加速度计Y轴零偏
    int16_t accel_z_offset; // 加速度计Z轴零偏
    uint8_t calibrated;     // 是否已校准
} Calibration_t;

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
// MPU6050函数
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadAccel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
HAL_StatusTypeDef MPU6050_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
HAL_StatusTypeDef MPU6050_CheckConnection(void);

// 角度计算函数
void MPU6050_ComputeAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           Angle_t *angle);

// 卡尔曼滤波器函数（新版本）
void Kalman_Init(Kalman_t *kalman);
float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt);
void MPU6050_ResetKalman(void);

// 校准函数
void MPU6050_Calibrate(void);
void MPU6050_CalibrateGyro(void);
void MPU6050_ResetAngles(void);
void MPU6050_SetCalibrationData(Calibration_t *calib);
void MPU6050_GetCalibrationData(Calibration_t *calib);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

