/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "main.h" // 必须第一个包含，确保类型定义和 HAL 函数声明
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 低电压保护参数
#define LOW_BATTERY_THRESHOLD   10.2f   // 3S电池: 3.4V/cell，留有降落裕量
#define LANDING_DURATION_MS     8000u   // 8秒平稳降落

// 自动降落与低电压相关变量（放在USER CODE BEGIN 0区域，避免CubeMX覆盖）
static uint8_t has_landed = 0;
static uint8_t low_battery_triggered = 0;
static uint8_t auto_landing_in_progress = 0;
static uint32_t landing_start_time = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	// OLED初始化
	SSD1306_Init();
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("OLED OK!");
	SSD1306_UpdateScreen();
	HAL_Delay(500);

	// 电机PWM初始化
	HAL_UART_Transmit(&huart1, (uint8_t*)"Motor Init...\r\n", 15, 10);
	Motor_Init();
	HAL_UART_Transmit(&huart1, (uint8_t*)"Motor OK!\r\n", 11, 10);

	// ESC解锁：发送最低油门3秒（参考Arduino代码sketch_apr25b.ino）
	// 如果ESC未校准过，需要先单独做一次校准：上电发2000us再降1000us
	HAL_UART_Transmit(&huart1, (uint8_t*)"ESC Arming 3s...\r\n", 18, 10);
	SSD1306_Clear();
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("ESC ARM");
	SSD1306_GotoXY(0, 1);
	SSD1306_Puts("Wait 3s...");
	SSD1306_UpdateScreen();
	Motor_SetAll(1000, 1000, 1000, 1000);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart1, (uint8_t*)"ESC Ready!\r\n", 12, 10);
	SSD1306_Clear();
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("ESC OK!");
	SSD1306_UpdateScreen();
	HAL_Delay(500);

	// MPU6050初始化
	uint8_t mpu_ok = 0;  // MPU6050初始化成功标志
	HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Initializing...\r\n", 24, 10);
	if (MPU6050_CheckConnection() != HAL_OK)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 NOT FOUND!\r\n", 20, 10);
	}
	else
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Detected!\r\n", 18, 10);
		if (MPU6050_Init() == HAL_OK)
		{
			mpu_ok = 1;
			HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 OK!\r\n", 13, 10);

			// 校准MPU6050（需要无人机水平放置并保持静止）
			HAL_UART_Transmit(&huart1, (uint8_t*)"Starting Calibration...\r\n", 25, 10);
			HAL_Delay(1000);
			MPU6050_Calibrate();
			HAL_Delay(500);

			// 重置角度为0（使用加速度计）
			// 注：当前使用卡尔曼滤波器，旧的互补滤波器代码已注释
			MPU6050_ResetAngles();
			HAL_UART_Transmit(&huart1, (uint8_t*)"Ready!\r\n", 9, 10);
		}
		else
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Init Failed!\r\n", 22, 10);
		}
	}

	// ADC变量
	uint32_t adc_value;
	uint32_t voltage_mv;  // 电压(毫伏)
	int16_t accel_x, accel_y, accel_z;  // 加速度计
	int16_t gyro_x, gyro_y, gyro_z;      // 陀螺仪
	Angle_t angle;  // 角度

	// 电机控制变量
	uint16_t motor1, motor2, motor3, motor4;
	uint16_t base_throttle = 1200;  // 基础油门
	int16_t pitch_adjust = 0;
	int16_t roll_adjust = 0;
	char uart_buffer[200];

	// 初始化稳定检测变量
	uint8_t init_phase = 1;  // 初始化阶段标志：1=初始化中，0=初始化完成
	uint16_t stable_count = 0;  // 稳定计数器
	uint16_t required_stable_samples = 30;  // 需要连续30次采样稳定(约1.5秒)
	uint32_t init_start_time = 0;  // 初始化开始时间
	float angle_tolerance = 1.0f;  // 角度容差范围（度）
	// OLED刷新控制（降低频率避免拖慢主循环）
	uint32_t oled_last_update = 0;
	uint32_t oled_update_interval = 200;  // 每200ms刷新一次OLED
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		uint32_t current_tick = HAL_GetTick();

		// 读取电池电压ADC原始值
		uint16_t bat_adc = ADC_ReadChannel(BATTERY_ADC_CHANNEL);
		float bat_v_adc = (float)bat_adc * ADC_REF_VOLTAGE / ADC_RESOLUTION;
		float bat_voltage = bat_v_adc * BATTERY_DIVIDER_RATIO;
		int32_t bat_int = (int32_t)bat_voltage;
		int32_t bat_frac = (int32_t)((bat_voltage - bat_int) * 100);
		if (bat_frac < 0) bat_frac = -bat_frac;

		// 使用ADC_ReadChannel读取电位器（PA0/CH0），避免与电池通道冲突
		adc_value = ADC_ReadChannel(ADC_CHANNEL_0);
		// 转换为电压(毫伏): 0-3300mV
		voltage_mv = adc_value * 3300 / 4095;

		// 使用电位器控制基础油门 (1000-2000us全范围)
		uint16_t pot_throttle = 1000 + (adc_value * 1000 / 4095);

		// ====== 低电压自动降落与禁止起飞逻辑 ======
		if (has_landed) {
			base_throttle = 1000;
			pitch_adjust = 0;
			roll_adjust = 0;
			Motor_SetAll(1000, 1000, 1000, 1000);
			init_phase = 0;
			sprintf(uart_buffer, "LANDED - NO RE-ARM. Bat: %ld.%02ldV\r\n", bat_int, bat_frac);
			HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
			HAL_Delay(100);
			continue;
		}

		if (bat_voltage < LOW_BATTERY_THRESHOLD && !low_battery_triggered && !init_phase) {
			low_battery_triggered = 1;
			auto_landing_in_progress = 1;
			landing_start_time = current_tick;
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nLOW BATTERY! Initiating Auto-Landing...\r\n", 44, 10);
		}

		if (auto_landing_in_progress) {
			uint32_t elapsed_landing_time = current_tick - landing_start_time;
			// 线性递减：从当前油门逐步降到1000
			float progress = (float)elapsed_landing_time / LANDING_DURATION_MS;
			if (progress > 1.0f) progress = 1.0f;
			base_throttle = (uint16_t)(pot_throttle * (1.0f - progress) + 1000.0f * progress);
			if (elapsed_landing_time >= LANDING_DURATION_MS) {
				auto_landing_in_progress = 0;
				has_landed = 1;
				HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nAuto-Landing Complete. System Disarmed.\r\n", 44, 10);
			}
		} else {
			base_throttle = pot_throttle;
		}

		// 读取MPU6050数据（仅在初始化成功时读取）
		if (mpu_ok &&
		    MPU6050_ReadAccel(&accel_x, &accel_y, &accel_z) == HAL_OK &&
		    MPU6050_ReadGyro(&gyro_x, &gyro_y, &gyro_z) == HAL_OK)
		{
			// 计算俯仰角和翻滚角（卡尔曼滤波）
			MPU6050_ComputeAngles(accel_x, accel_y, accel_z,
			                       gyro_x, gyro_y, gyro_z, &angle);

			// 初始化阶段：等待角度稳定
			if (init_phase)
			{
				if (init_start_time == 0)
				{
					init_start_time = current_tick;
				}

				uint32_t elapsed_time = current_tick - init_start_time;
				if (elapsed_time > 30000 && angle_tolerance == 1.0f)
				{
					angle_tolerance = 2.0f;
					HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nTIMEOUT - Relaxing tolerance to +/- 2 degrees\r\n", 49, 10);
					stable_count = 0;
				}
				if (elapsed_time > 60000 && angle_tolerance == 2.0f)
				{
					angle_tolerance = 3.0f;
					HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nTIMEOUT - Relaxing tolerance to +/- 3 degrees\r\n", 49, 10);
					stable_count = 0;
				}

				if (angle.pitch >= -angle_tolerance && angle.pitch <= angle_tolerance &&
				    angle.roll >= -angle_tolerance && angle.roll <= angle_tolerance)
				{
					stable_count++;
					if (stable_count >= required_stable_samples)
					{
						init_phase = 0;
						HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n=== INITIALIZATION COMPLETE ===\r\n", 34, 10);
						HAL_UART_Transmit(&huart1, (uint8_t*)"Ready for flight control!\r\n", 27, 10);
					}
				}
				else
				{
					stable_count = 0;
				}

				// 初始化阶段只打印角度，不控制电机
				{
					int32_t tol_i = (int32_t)angle_tolerance;
					int32_t tol_f = (int32_t)((angle_tolerance - tol_i) * 10);
					if (tol_f < 0) tol_f = -tol_f;
					sprintf(uart_buffer, "INIT PHASE - P: %ld.%02lu R: %ld.%02lu | Stable: %3d/%3d | Tol: %ld.%ld deg | Time: %lu s\r\n",
							(int32_t)angle.pitch, (uint32_t)((angle.pitch >= 0 ? angle.pitch : -angle.pitch) * 100) % 100,
							(int32_t)angle.roll, (uint32_t)((angle.roll >= 0 ? angle.roll : -angle.roll) * 100) % 100,
							stable_count, required_stable_samples, tol_i, tol_f, elapsed_time / 1000);
				}
				HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
			}
			else
			{
				// 初始化完成，开始飞行控制
				if (fabsf(angle.pitch) < ANGLE_DEADZONE) angle.pitch = 0.0f;
				if (fabsf(angle.roll) < ANGLE_DEADZONE) angle.roll = 0.0f;

				pitch_adjust = (int16_t)(angle.pitch * 2.0f);
				roll_adjust = (int16_t)(angle.roll * 2.0f);
				if (pitch_adjust > 500) pitch_adjust = 500;
				if (pitch_adjust < -500) pitch_adjust = -500;
				if (roll_adjust > 500) roll_adjust = 500;
				if (roll_adjust < -500) roll_adjust = -500;

				motor1 = base_throttle + pitch_adjust - roll_adjust;
				motor2 = base_throttle + pitch_adjust + roll_adjust;
				motor3 = base_throttle - pitch_adjust - roll_adjust;
				motor4 = base_throttle - pitch_adjust + roll_adjust;

				if (motor1 < 1000) motor1 = 1000;
				if (motor1 > 2000) motor1 = 2000;
				if (motor2 < 1000) motor2 = 1000;
				if (motor2 > 2000) motor2 = 2000;
				if (motor3 < 1000) motor3 = 1000;
				if (motor3 > 2000) motor3 = 2000;
				if (motor4 < 1000) motor4 = 1000;
				if (motor4 > 2000) motor4 = 2000;

				Motor_SetAll(motor1, motor2, motor3, motor4);

				sprintf(uart_buffer, "BAT:%ld.%02ldV POT:%lu.%02luV P:%ld.%02lu R:%ld.%02lu M:%4d %4d %4d %4d\r\n",
				        bat_int, bat_frac,
				        voltage_mv / 1000, (voltage_mv % 1000) / 100,
				        (int32_t)angle.pitch, (uint32_t)((angle.pitch >= 0 ? angle.pitch : -angle.pitch) * 100) % 100,
				        (int32_t)angle.roll, (uint32_t)((angle.roll >= 0 ? angle.roll : -angle.roll) * 100) % 100,
				        motor1, motor2, motor3, motor4);
				HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);

				// OLED显示（每200ms刷新一次，避免拖慢主循环）
				uint32_t now = HAL_GetTick();
				if (now - oled_last_update >= oled_update_interval)
				{
					oled_last_update = now;
					SSD1306_Clear();
					SSD1306_GotoXY(0, 0);
					SSD1306_Printf("BAT:%d.%02dV", (int)bat_int, (int)bat_frac);
					SSD1306_GotoXY(0, 1);
					{
						int32_t sp = (int32_t)angle.pitch;
						int32_t sp_f = (int32_t)((angle.pitch >= 0 ? angle.pitch : -angle.pitch) * 100) % 100;
						int32_t sr = (int32_t)angle.roll;
						int32_t sr_f = (int32_t)((angle.roll >= 0 ? angle.roll : -angle.roll) * 100) % 100;
						char p_sign = (angle.pitch >= 0) ? '+' : '-';
						char r_sign = (angle.roll >= 0) ? '+' : '-';
						SSD1306_Printf("P:%c%d.%02d R:%c%d.%02d",
								p_sign, (int)sp, (int)sp_f,
								r_sign, (int)sr, (int)sr_f);
					}
					SSD1306_GotoXY(0, 2);
					SSD1306_Printf("M1:%d M2:%d", (int)motor1, (int)motor2);
					SSD1306_GotoXY(0, 3);
					SSD1306_Printf("M3:%d M4:%d", (int)motor3, (int)motor4);
					SSD1306_UpdateScreen();
				}
			}
		}
		else if (mpu_ok)
		{
			// MPU6050曾初始化成功但读取失败
			sprintf(uart_buffer, "ADC: %4lu, V: %lu.%02luV | MPU6050 Read Error\r\n",
					adc_value, voltage_mv / 1000, (voltage_mv % 1000) / 10);
			HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
		}

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
