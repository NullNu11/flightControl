/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include <string.h>

// 外部UART句柄声明（用于校准输出）
extern UART_HandleTypeDef huart1;

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// 校准数据存储
static Calibration_t calibration = {0};

// 角度重置标志
static uint8_t need_angle_reset = 0;

// 卡尔曼滤波器实例（新版本）
static Kalman_t kalman_pitch = {0};  // 俯仰角卡尔曼滤波器
static Kalman_t kalman_roll = {0};   // 翻滚角卡尔曼滤波器

// 快速计算平方根（牛顿迭代法）
static float fast_sqrt(float x)
{
    float half = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - half * x * x);
    x = x * (1.5f - half * x * x);
    return x;
}

// 快速计算反正切（简化的atan2近似）
static float fast_atan2(float y, float x)
{
    if (x > 0)
    {
        if (y > 0)
        {
            if (x > y)
                return y / x * 45.0f;
            else
                return 90.0f - x / y * 45.0f;
        }
        else
        {
            y = -y;
            if (x > y)
                return -y / x * 45.0f;
            else
                return -90.0f + x / y * 45.0f;
        }
    }
    else
    {
        x = -x;
        if (y > 0)
        {
            if (x > y)
                return 180.0f - y / x * 45.0f;
            else
                return 90.0f + x / y * 45.0f;
        }
        else
        {
            y = -y;
            if (x > y)
                return -180.0f + y / x * 45.0f;
            else
                return -90.0f - x / y * 45.0f;
        }
    }
}

// ============================================================================
// 卡尔曼滤波器实现（新版本 - 一维卡尔曼滤波）
// ============================================================================

/**
 * @brief 初始化卡尔曼滤波器
 * @param kalman 卡尔曼滤波器结构体指针
 */
void Kalman_Init(Kalman_t *kalman)
{
    // 初始化状态估计
    kalman->angle = 0.0f;       // 初始角度为0
    kalman->bias = 0.0f;        // 初始偏移为0

    // 初始化误差协方差矩阵（对角矩阵）
    // P = [P00 P01]
    //     [P10 P11]
    kalman->P[0][0] = 1.0f;     // 角度误差方差
    kalman->P[0][1] = 0.0f;     // 角度和偏移的协方差
    kalman->P[1][0] = 0.0f;     // 偏移和角度的协方差
    kalman->P[1][1] = 0.1f;     // 偏移误差方差
}

/**
 * @brief 卡尔曼滤波器更新（一维卡尔曼滤波）
 * @param kalman 卡尔曼滤波器结构体指针
 * @param newAngle 加速度计测量的角度（弧度）
 * @param newRate 陀螺仪测量的角速度（度/秒）
 * @param dt 时间间隔（秒）
 * @return 估计的角度（弧度）
 */
float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt)
{
    // 预测步骤（Prediction Step）

    // 1. 状态预测
    // angle = angle + (gyro_rate - bias) * dt
    // gyro_rate已经去掉了偏移，所以直接积分
    float rate = newRate - kalman->bias;
    kalman->angle += rate * dt;

    // 2. 协方差预测
    // P = A * P * A^T + Q
    // 其中 A = [1  -dt]
    //          [0   1 ]
    //      Q = [Q_ANGLE   0     ]
    //          [0         Q_BIAS]
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + KALMAN_Q_ANGLE);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += KALMAN_Q_BIAS * dt;

    // 更新步骤（Update Step）

    // 1. 计算卡尔曼增益
    // K = P * H^T * (H * P * H^T + R)^(-1)
    // 其中 H = [1  0]（只测量角度）
    //      R = R_MEASURE
    float S = kalman->P[0][0] + KALMAN_R_MEASURE;  // 创新协方差
    float K[2];  // 卡尔曼增益向量 2x1
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // 2. 状态更新
    // angle = angle + K * (measurement - predicted_angle)
    float y = newAngle - kalman->angle;  // 创新（innovation）
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // 3. 协方差更新
    // P = (I - K * H) * P
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

/**
 * @brief 重置卡尔曼滤波器
 */
void MPU6050_ResetKalman(void)
{
    Kalman_Init(&kalman_pitch);
    Kalman_Init(&kalman_roll);
}

// ============================================================================
// 互补滤波器实现（旧版本 - 已注释，仅供参考学习）
// ============================================================================

/*
// 计算俯仰角和翻滚角（使用互补滤波器）
void MPU6050_ComputeAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           Angle_t *angle)
{
    static float pitch = 0.0f, roll = 0.0f;

    gyro_x -= calibration.gyro_x_offset;
    gyro_y -= calibration.gyro_y_offset;
    gyro_z -= calibration.gyro_z_offset;

    // 应用加速度计偏移校准
    accel_x -= calibration.accel_x_offset;
    accel_y -= calibration.accel_y_offset;
    accel_z -= calibration.accel_z_offset;

    float ax = (float)accel_x / 8192.0f;
    float ay = (float)accel_y / 8192.0f;
    float az = (float)accel_z / 8192.0f;

    float gx = (float)gyro_x / 65.5f;
    float gy = (float)gyro_y / 65.5f;
    float gz = (float)gyro_z / 65.5f;

    if (need_angle_reset)
    {
        pitch = fast_atan2(ax, fast_sqrt(ay * ay + az * az)) * 3.1415926f / 180.0f;
        roll  = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 3.1415926f / 180.0f;
        need_angle_reset = 0;
    }

    float pitch_accel = fast_atan2(ax, fast_sqrt(ay * ay + az * az)) * 3.1415926f / 180.0f;
    float roll_accel  = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 3.1415926f / 180.0f;

    // 互补滤波器公式
    // angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
    // ALPHA: 陀螺仪权重（短期准确）
    // (1-ALPHA): 加速度计权重（长期稳定）
    pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * pitch_accel;
    roll  = ALPHA * (roll  + gy * DT) + (1.0f - ALPHA) * roll_accel;

    angle->pitch = pitch * 180.0f / 3.1415926f;
    angle->roll  = roll  * 180.0f / 3.1415926f;
}
*/

// ============================================================================
// 卡尔曼滤波器版本（新版本 - 当前使用）
// ============================================================================

// MPU6050写单个寄存器
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// MPU6050读单个寄存器
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

// MPU6050检查连接
HAL_StatusTypeDef MPU6050_CheckConnection(void)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status = MPU6050_ReadReg(MPU6050_WHO_AM_I, &who_am_i);
    if (status == HAL_OK && who_am_i == 0x68)
    {
        return HAL_OK;
    }
    return HAL_ERROR;
}

// MPU6050初始化
HAL_StatusTypeDef MPU6050_Init(void)
{
    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(100);
    if (MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x19) != HAL_OK)
        return HAL_ERROR;
    if (MPU6050_WriteReg(MPU6050_CONFIG, 0x03) != HAL_OK)
        return HAL_ERROR;
    if (MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08) != HAL_OK)
        return HAL_ERROR;
    if (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x08) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

// 读取加速度计数据 (原始值)
HAL_StatusTypeDef MPU6050_ReadAccel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    uint8_t data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}

// 读取陀螺仪数据 (原始值)
HAL_StatusTypeDef MPU6050_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43,
                         I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    *gyro_x = (int16_t)((data[0] << 8) | data[1]);
    *gyro_y = (int16_t)((data[2] << 8) | data[3]);
    *gyro_z = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}

// ============================================================================
// 互补滤波器版本（旧版本 - 已注释，仅供参考学习）
// ============================================================================

/*
// 计算俯仰角和翻滚角（使用互补滤波器）
void MPU6050_ComputeAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           Angle_t *angle)
{
    static float pitch = 0.0f, roll = 0.0f;

    gyro_x -= calibration.gyro_x_offset;
    gyro_y -= calibration.gyro_y_offset;
    gyro_z -= calibration.gyro_z_offset;

    // 应用加速度计偏移校准
    accel_x -= calibration.accel_x_offset;
    accel_y -= calibration.accel_y_offset;
    accel_z -= calibration.accel_z_offset;

    float ax = (float)accel_x / 8192.0f;
    float ay = (float)accel_y / 8192.0f;
    float az = (float)accel_z / 8192.0f;

    float gx = (float)gyro_x / 65.5f;
    float gy = (float)gyro_y / 65.5f;
    float gz = (float)gyro_z / 65.5f;

    if (need_angle_reset)
    {
        pitch = fast_atan2(ax, fast_sqrt(ay * ay + az * az)) * 3.1415926f / 180.0f;
        roll  = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 3.1415926f / 180.0f;
        need_angle_reset = 0;
    }

    float pitch_accel = fast_atan2(ax, fast_sqrt(ay * ay + az * az)) * 3.1415926f / 180.0f;
    float roll_accel  = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 3.1415926f / 180.0f;

    // 互补滤波器公式
    // angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
    // ALPHA: 陀螺仪权重（短期准确）
    // (1-ALPHA): 加速度计权重（长期稳定）
    pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * pitch_accel;
    roll  = ALPHA * (roll  + gy * DT) + (1.0f - ALPHA) * roll_accel;

    angle->pitch = pitch * 180.0f / 3.1415926f;
    angle->roll  = roll  * 180.0f / 3.1415926f;
}
*/

// ============================================================================
// 卡尔曼滤波器版本（新版本 - 当前使用）
// ============================================================================

// 计算俯仰角和翻滚角（使用卡尔曼滤波器）
void MPU6050_ComputeAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           Angle_t *angle)
{
    // 测量实际采样间隔
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (last_time == 0) ? 0.05f : (float)(now - last_time) / 1000.0f;
    last_time = now;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f)   dt = 0.5f;

    // 应用陀螺仪偏移校准
    gyro_x -= calibration.gyro_x_offset;
    gyro_y -= calibration.gyro_y_offset;
    gyro_z -= calibration.gyro_z_offset;

    // 应用加速度计偏移校准
    accel_x -= calibration.accel_x_offset;
    accel_y -= calibration.accel_y_offset;
    accel_z -= calibration.accel_z_offset;

    // 转换加速度计数据（转换为g）
    float ax = (float)accel_x / 8192.0f;
    float ay = (float)accel_y / 8192.0f;
    float az = (float)accel_z / 8192.0f;

    // 转换陀螺仪数据（转换为度/秒）
    float gx = (float)gyro_x / 65.5f;
    float gy = (float)gyro_y / 65.5f;
    float gz = (float)gyro_z / 65.5f;

    // 加速度计角度低通滤波（在送入卡尔曼之前先平滑，消除原始噪声尖刺）
    static float pitch_accel_smooth = 0.0f;
    static float roll_accel_smooth = 0.0f;
    float pitch_accel_raw = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
    float roll_accel_raw  = atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2957795f;
    pitch_accel_smooth = 0.4f * pitch_accel_raw + 0.6f * pitch_accel_smooth;
    roll_accel_smooth  = 0.4f * roll_accel_raw  + 0.6f * roll_accel_smooth;

    // 角度重置处理：保存标志再清零，确保后续输出平滑也能正确重置
    uint8_t do_reset = need_angle_reset;
    if (do_reset)
    {
        Kalman_Init(&kalman_pitch);
        Kalman_Init(&kalman_roll);
        kalman_pitch.angle = pitch_accel_smooth;
        kalman_roll.angle  = roll_accel_smooth;
        pitch_accel_smooth = pitch_accel_raw;
        roll_accel_smooth  = roll_accel_raw;
        need_angle_reset = 0;
    }

    // 使用卡尔曼滤波器更新角度（全程使用度数，实际dt）
    float kalman_pitch_out = Kalman_Update(&kalman_pitch, pitch_accel_smooth, gx, dt);
    float kalman_roll_out  = Kalman_Update(&kalman_roll,  roll_accel_smooth,  gy, dt);

    // 零点偏移：记录静止时的角度作为偏移量，后续输出减去它
    static float pitch_offset = 0.0f;
    static float roll_offset  = 0.0f;
    if (do_reset)
    {
        pitch_offset = kalman_pitch_out;
        roll_offset  = kalman_roll_out;
    }

    // 减去零点偏移，使静止时输出为0
    float pitch_centered = kalman_pitch_out - pitch_offset;
    float roll_centered  = kalman_roll_out  - roll_offset;

    // 输出指数平滑（在归零后的值上平滑，确保零点稳定）
    static float output_pitch = 0.0f;
    static float output_roll  = 0.0f;
    if (do_reset) {
        output_pitch = pitch_centered;
        output_roll  = roll_centered;
    }
    output_pitch = SMOOTH_FACTOR * pitch_centered + (1.0f - SMOOTH_FACTOR) * output_pitch;
    output_roll  = SMOOTH_FACTOR * roll_centered  + (1.0f - SMOOTH_FACTOR) * output_roll;

    angle->pitch = output_pitch;
    angle->roll  = output_roll;
}

// 陀螺仪零偏校准
void MPU6050_CalibrateGyro(void)
{
    int16_t gx, gy, gz;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t samples = 500;
    uint16_t i;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Gyro Calibration...\r\n", 22, 1000);

    for (i = 0; i < samples; i++)
    {
        if (MPU6050_ReadGyro(&gx, &gy, &gz) == HAL_OK)
        {
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
        }
        HAL_Delay(2);
    }

    calibration.gyro_x_offset = (int16_t)(sum_gx / samples);
    calibration.gyro_y_offset = (int16_t)(sum_gy / samples);
    calibration.gyro_z_offset = (int16_t)(sum_gz / samples);
    calibration.calibrated = 1;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Gyro Calibration Done!\r\n", 25, 1000);
}

// 完整校准（陀螺仪和加速度计）
void MPU6050_Calibrate(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t samples = 500;
    uint16_t i;

    HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Calibration...\r\n", 23, 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Please keep the drone LEVEL and STILL!\r\n", 41, 1000);
    HAL_Delay(2000);

    for (i = 0; i < samples; i++)
    {
        if (MPU6050_ReadAccel(&ax, &ay, &az) == HAL_OK &&
            MPU6050_ReadGyro(&gx, &gy, &gz) == HAL_OK)
        {
            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
        }
        HAL_Delay(2);
    }

    calibration.gyro_x_offset = (int16_t)(sum_gx / samples);
    calibration.gyro_y_offset = (int16_t)(sum_gy / samples);
    calibration.gyro_z_offset = (int16_t)(sum_gz / samples);

    // 完整的加速度计校准（X、Y、Z三个轴）
    calibration.accel_x_offset = (int16_t)(sum_ax / samples);
    calibration.accel_y_offset = (int16_t)(sum_ay / samples);
    calibration.accel_z_offset = (int16_t)(sum_az / samples - 8192);
    calibration.calibrated = 1;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibration Done!\r\n", 18, 1000);

    char buffer[100];
    sprintf(buffer, "Gyro Offset: X=%d, Y=%d, Z=%d\r\n",
            calibration.gyro_x_offset, calibration.gyro_y_offset, calibration.gyro_z_offset);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    sprintf(buffer, "Accel Offset: X=%d, Y=%d, Z=%d\r\n",
            calibration.accel_x_offset, calibration.accel_y_offset, calibration.accel_z_offset);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

// 重置角度
void MPU6050_ResetAngles(void)
{
    need_angle_reset = 1;
    // 重置卡尔曼滤波器
    MPU6050_ResetKalman();
    HAL_UART_Transmit(&huart1, (uint8_t*)"Angles Reset!\r\n", 15, 1000);
}

// 设置校准数据
void MPU6050_SetCalibrationData(Calibration_t *calib)
{
    calibration = *calib;
}

// 获取校准数据
void MPU6050_GetCalibrationData(Calibration_t *calib)
{
    *calib = calibration;
}

/* USER CODE END 1 */


