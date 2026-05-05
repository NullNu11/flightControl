/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0 (电位器)
    PA1          ------> ADC1_IN1 (电池电压检测)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1          ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// 外部UART句柄声明
extern UART_HandleTypeDef huart1;

/**
 * @brief 读取指定ADC通道的值
 * @param channel ADC通道（如ADC_CHANNEL_0, ADC_CHANNEL_1等）
 * @return ADC原始值（0-4095）
 */
uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_sum = 0;

  // TODO: 优化为DMA或连续转换，减少stop/start次数

    // 先停止ADC，避免状态冲突
    HAL_ADC_Stop(&hadc1);

    // 配置ADC通道
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0;
    }

    // 多次采样取平均值，降低噪声
    for (uint8_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return 0;
        }

        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            adc_sum += HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            HAL_ADC_Stop(&hadc1);
            return 0;
        }
    }

    HAL_ADC_Stop(&hadc1);

    return (uint16_t)(adc_sum / ADC_OVERSAMPLE_COUNT);
}

/**
 * @brief 读取电池电压
 * @return 电池电压（伏特）
 * @note 假设使用分压电路，需要根据实际电阻值调整BATTERY_DIVIDER_RATIO
 */
float Battery_ReadVoltage(void)
{
    // 读取电池电压ADC通道（含过采样平均）
    uint16_t adc_value = ADC_ReadChannel(BATTERY_ADC_CHANNEL);

    // 计算实际电压
    // V_adc = adc_value * ADC_REF_VOLTAGE / ADC_RESOLUTION
    // V_battery = V_adc * BATTERY_DIVIDER_RATIO * BATTERY_CORRECTION
    float v_adc = (float)adc_value * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    float v_battery = v_adc * BATTERY_DIVIDER_RATIO * BATTERY_CORRECTION;

    return v_battery;
}

/**
 * @brief 根据电压计算电池百分比
 * @param voltage 电池电压
 * @return 电池百分比（0-100）
 */
uint8_t Battery_GetPercentage(float voltage)
{
    if (voltage >= BATTERY_VOLTAGE_MAX)
    {
        return 100;
    }
    else if (voltage <= BATTERY_VOLTAGE_MIN)
    {
        return 0;
    }
    else
    {
        // 线性插值计算百分比
        float percentage = (voltage - BATTERY_VOLTAGE_MIN) /
                          (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f;
        return (uint8_t)percentage;
    }
}

/**
 * @brief 打印电池状态到串口
 */
void Battery_PrintStatus(void)
{
    // 先读取ADC原始值
    uint16_t adc_raw = ADC_ReadChannel(BATTERY_ADC_CHANNEL);

    // 计算电压
    float v_adc = (float)adc_raw * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    float voltage = v_adc * BATTERY_DIVIDER_RATIO;
    uint8_t percentage = Battery_GetPercentage(voltage);

    char buffer[120];

    // 判断电池状态
    const char* status;
    if (voltage >= BATTERY_VOLTAGE_MAX * 0.9f)
    {
        status = "FULL";
    }
    else if (voltage >= BATTERY_VOLTAGE_MIN)
    {
        status = "OK";
    }
    else
    {
        status = "LOW";
    }

    // 格式化输出（含ADC原始值用于调试）
    int32_t v_int = (int32_t)voltage;
    int32_t v_frac = (int32_t)((voltage - v_int) * 100);
    if (v_frac < 0) v_frac = -v_frac;

    sprintf(buffer, "BAT: %ld.%02ldV (%d%%) [%s] ADC=%d\r\n",
            v_int, v_frac, percentage, status, adc_raw);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/* USER CODE END 1 */
