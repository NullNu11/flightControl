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
#include "ibus.h"
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
#define FLIGHT_CONTROL_DT      0.0025f // 飞控采样周期: 1/400Hz = 2.5ms

// ====== 飞控共享变量（ISR与主循环之间共享，必须volatile） ======
volatile uint8_t  g_mpu_ok = 0;               // MPU6050初始化成功标志
volatile Angle_t  g_angle = {0, 0};           // 最新角度（ISR写，主循环读）
volatile int16_t  g_pitch_adjust = 0;         // 俯仰调节量
volatile int16_t  g_roll_adjust = 0;          // 翻滚调节量
volatile uint16_t g_motor1 = 1000;            // 电机1脉宽
volatile uint16_t g_motor2 = 1000;            // 电机2脉宽
volatile uint16_t g_motor3 = 1000;            // 电机3脉宽
volatile uint16_t g_motor4 = 1000;            // 电机4脉宽
volatile uint8_t  g_mpu_read_ok = 0;          // 最新一次MPU读取是否成功

// 主循环写，ISR读
volatile uint16_t g_base_throttle = 1000;     // 基础油门（电位器/降落逻辑控制）
volatile uint8_t  g_init_phase = 1;           // 初始化阶段标志
volatile uint8_t  g_has_landed = 0;           // 已降落标志
volatile uint8_t  g_auto_landing_in_progress = 0; // 自动降落中
volatile uint32_t g_landing_start_time = 0;   // 降落开始时间
volatile uint8_t  g_low_battery_triggered = 0;// 低电压触发标志

// 油门保护参数
#define THROTTLE_MAX_LIMIT    1500            // 油门上限限制
#define THROTTLE_RAMP_MS      2000u           // 启动防冲缓升时间（2秒）
volatile uint32_t g_arm_start_time = 0;       // 解锁时刻记录

// RC遥控器共享变量（主循环写，ISR读）
volatile float    g_rc_pitch_target = 0.0f;   // 目标俯仰角（度）
volatile float    g_rc_roll_target = 0.0f;    // 目标翻滚角（度）
volatile uint8_t  g_rc_connected = 0;          // RC连接状态
volatile uint8_t  g_rc_armed = 0;              // RC解锁状态

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
	if (MPU6050_CheckConnection() != HAL_OK)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 NOT FOUND!\r\n", 20, 10);
	}
	else
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Detected!\r\n", 18, 10);
		if (MPU6050_Init() == HAL_OK)
		{
			g_mpu_ok = 1;
			HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 OK!\r\n", 13, 10);

			// 校准MPU6050（需要无人机水平放置并保持静止）
			HAL_UART_Transmit(&huart1, (uint8_t*)"Starting Calibration...\r\n", 25, 10);
			HAL_Delay(1000);
			MPU6050_Calibrate();
			HAL_Delay(500);

			// 重置角度为0（使用加速度计）
			MPU6050_ResetAngles();
			HAL_UART_Transmit(&huart1, (uint8_t*)"Ready!\r\n", 9, 10);
		}
		else
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Init Failed!\r\n", 22, 10);
		}
	}

	// 启动飞控定时器中断（TIM3 @ 400Hz）
	FlightControl_Start();
	HAL_UART_Transmit(&huart1, (uint8_t*)"Flight Control ISR Started @ 400Hz\r\n", 36, 10);

	// 初始化i-BUS接收（USART2 DMA + IDLE中断）
	IBUS_Init();
	HAL_UART_Transmit(&huart1, (uint8_t*)"i-BUS Init OK (PA3, 115200 8E1)\r\n", 33, 10);

	// 主循环局部变量
	char uart_buffer[200];

	// 初始化稳定检测变量
	uint16_t stable_count = 0;
	uint16_t required_stable_samples = 30;
	uint32_t init_start_time = 0;
	float angle_tolerance = 1.0f;
	// OLED刷新控制
	uint32_t oled_last_update = 0;
	uint32_t oled_update_interval = 200;
	// i-BUS调试打印控制
	uint32_t ibus_dbg_last = 0;
	uint32_t ibus_dbg_interval = 5000;   // 每5秒打印一次（减少阻塞）
	// 串口状态行输出控制（降低频率以减少主循环延迟）
	uint32_t status_last_print = 0;
	uint32_t status_print_interval = 100; // 每100ms打印一次状态行
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

		// i-BUS帧解析与RC输入处理（优先级最高，不受调试打印影响）
		IBUS_Update();
		g_rc_connected = IBUS_IsConnected();
		if (g_rc_connected) {
			uint16_t ch_roll     = IBUS_GetChannel(IBUS_CH_ROLL);
			uint16_t ch_pitch    = IBUS_GetChannel(IBUS_CH_PITCH);
			uint16_t ch_throttle = IBUS_GetChannel(IBUS_CH_THROTTLE);
			uint16_t ch_arm      = IBUS_GetChannel(IBUS_CH_ARM);

			// 摇杆映射到目标角度: 中心=0°, 满偏=±30°
			g_rc_roll_target  = ((float)ch_roll  - IBUS_CH_CENTER) /
			                    (IBUS_CH_MAX - IBUS_CH_CENTER) * RC_MAX_ANGLE;
			g_rc_pitch_target = ((float)ch_pitch - IBUS_CH_CENTER) /
			                    (IBUS_CH_MAX - IBUS_CH_CENTER) * RC_MAX_ANGLE;

			// 解锁开关: CH5 > 1500 = 解锁
			uint8_t prev_armed = g_rc_armed;
			g_rc_armed = (ch_arm > 1500) ? 1 : 0;

			// 检测解锁上升沿，记录时刻
			if (g_rc_armed && !prev_armed)
			{
				g_arm_start_time = current_tick;
			}

			// 油门处理（仅解锁时生效）
			if (g_rc_armed) {
				// 油门行程上限限制
				if (ch_throttle > THROTTLE_MAX_LIMIT) ch_throttle = THROTTLE_MAX_LIMIT;

				// 启动防冲：油门从1000缓升到目标值
				uint32_t armed_elapsed = current_tick - g_arm_start_time;
				if (armed_elapsed < THROTTLE_RAMP_MS)
				{
					// 线性缓升: ramp_ratio 从 0→1
					float ramp_ratio = (float)armed_elapsed / (float)THROTTLE_RAMP_MS;
					uint16_t ramp_throttle = 1000 + (uint16_t)((float)(ch_throttle - 1000) * ramp_ratio);
					g_base_throttle = ramp_throttle;
				}
				else
				{
					g_base_throttle = ch_throttle;
				}
			} else {
				g_base_throttle = 1000;
			}
		} else {
			// RC未连接：目标角度归零，不解锁
			g_rc_roll_target  = 0.0f;
			g_rc_pitch_target = 0.0f;
			g_rc_armed = 0;
			g_base_throttle = 1000;
		}

		// i-BUS调试信息输出（每2秒）—— 放在RC处理之后，避免阻塞影响连接检测
		if (current_tick - ibus_dbg_last >= ibus_dbg_interval)
		{
			ibus_dbg_last = current_tick;
			IBUS_DebugPrint();
		}

		// ====== 低电压自动降落与禁止起飞逻辑 ======
		if (g_has_landed) {
			g_base_throttle = 1000;
			sprintf(uart_buffer, "LANDED - NO RE-ARM. Bat: %ld.%02ldV\r\n", bat_int, bat_frac);
			HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
			HAL_Delay(100);
			continue;
		}

		if (bat_voltage < LOW_BATTERY_THRESHOLD && !g_low_battery_triggered && !g_init_phase) {
			g_low_battery_triggered = 1;
			g_auto_landing_in_progress = 1;
			g_landing_start_time = current_tick;
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nLOW BATTERY! Initiating Auto-Landing...\r\n", 44, 10);
		}

		if (g_auto_landing_in_progress) {
			uint32_t elapsed_landing_time = current_tick - g_landing_start_time;
			float progress = (float)elapsed_landing_time / LANDING_DURATION_MS;
			if (progress > 1.0f) progress = 1.0f;
			uint16_t landing_throttle = IBUS_GetChannel(IBUS_CH_THROTTLE);
			g_base_throttle = (uint16_t)(landing_throttle * (1.0f - progress) + 1000.0f * progress);
			if (elapsed_landing_time >= LANDING_DURATION_MS) {
				g_auto_landing_in_progress = 0;
				g_has_landed = 1;
				HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nAuto-Landing Complete. System Disarmed.\r\n", 44, 10);
			}
		}

		// ====== 初始化阶段：等待角度稳定 ======
		if (g_init_phase)
		{
			Angle_t local_angle = g_angle;

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

			if (local_angle.pitch >= -angle_tolerance && local_angle.pitch <= angle_tolerance &&
			    local_angle.roll >= -angle_tolerance && local_angle.roll <= angle_tolerance)
			{
				stable_count++;
				if (stable_count >= required_stable_samples)
				{
					g_init_phase = 0;
					HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n=== INITIALIZATION COMPLETE ===\r\n", 34, 10);
					HAL_UART_Transmit(&huart1, (uint8_t*)"Ready for flight control!\r\n", 27, 10);
				}
			}
			else
			{
				stable_count = 0;
			}

			int32_t tol_i = (int32_t)angle_tolerance;
			int32_t tol_f = (int32_t)((angle_tolerance - tol_i) * 10);
			if (tol_f < 0) tol_f = -tol_f;
			// 初始化阶段状态输出（每500ms一次）
			if (current_tick - status_last_print >= 500)
			{
				status_last_print = current_tick;
				sprintf(uart_buffer, "INIT P:%ld.%02lu R:%ld.%02lu S:%3d/%3d T:%lus\r\n",
						(int32_t)local_angle.pitch, (uint32_t)((local_angle.pitch >= 0 ? local_angle.pitch : -local_angle.pitch) * 100) % 100,
						(int32_t)local_angle.roll, (uint32_t)((local_angle.roll >= 0 ? local_angle.roll : -local_angle.roll) * 100) % 100,
						stable_count, required_stable_samples, elapsed_time / 1000);
				HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
			}
		}
		else
		{
			// 飞行控制已在ISR中完成，主循环仅做数据输出
			Angle_t local_angle = g_angle;
			uint16_t m1 = g_motor1, m2 = g_motor2, m3 = g_motor3, m4 = g_motor4;

			// RC状态指示
			const char *rc_status = g_rc_connected ? (g_rc_armed ? "ARM" : "DIS") : "NORC";

			// 状态行串口输出（每100ms一次，减少主循环阻塞）
			if (current_tick - status_last_print >= status_print_interval)
			{
				status_last_print = current_tick;
				sprintf(uart_buffer, "BAT:%ld.%02ldV %s P:%ld.%02lu R:%ld.%02lu T:%d M:%4d %4d %4d %4d\r\n",
				        bat_int, bat_frac, rc_status,
				        (int32_t)local_angle.pitch, (uint32_t)((local_angle.pitch >= 0 ? local_angle.pitch : -local_angle.pitch) * 100) % 100,
				        (int32_t)local_angle.roll, (uint32_t)((local_angle.roll >= 0 ? local_angle.roll : -local_angle.roll) * 100) % 100,
				        g_base_throttle,
				        m1, m2, m3, m4);
				HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 10);
			}

			// OLED显示（每200ms刷新一次）
			uint32_t now = HAL_GetTick();
			if (now - oled_last_update >= oled_update_interval)
			{
				oled_last_update = now;
				SSD1306_Clear();
				SSD1306_GotoXY(0, 0);
				SSD1306_Printf("BAT:%d.%02dV %s", (int)bat_int, (int)bat_frac, rc_status);
				SSD1306_GotoXY(0, 1);
				{
					int32_t sp = (int32_t)local_angle.pitch;
					int32_t sp_f = (int32_t)((local_angle.pitch >= 0 ? local_angle.pitch : -local_angle.pitch) * 100) % 100;
					int32_t sr = (int32_t)local_angle.roll;
					int32_t sr_f = (int32_t)((local_angle.roll >= 0 ? local_angle.roll : -local_angle.roll) * 100) % 100;
					char p_sign = (local_angle.pitch >= 0) ? '+' : '-';
					char r_sign = (local_angle.roll >= 0) ? '+' : '-';
					SSD1306_Printf("P:%c%d.%02d R:%c%d.%02d",
							p_sign, (int)sp, (int)sp_f,
							r_sign, (int)sr, (int)sr_f);
				}
				SSD1306_GotoXY(0, 2);
				SSD1306_Printf("M1:%d M2:%d", (int)m1, (int)m2);
				SSD1306_GotoXY(0, 3);
				SSD1306_Printf("M3:%d M4:%d", (int)m3, (int)m4);
				SSD1306_UpdateScreen();
			}
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

/**
 * @brief 启动飞控定时器中断（TIM3 @ 400Hz）
 */
void FlightControl_Start(void)
{
    // 使能TIM3 NVIC中断
    HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    // 使能TIM3更新中断
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

/**
 * @brief 飞控定时器中断回调（400Hz）
 *        读取MPU6050、计算角度、控制电机输出
 *        由 HAL_TIM_PeriodElapsedCallback 调用
 */
void FlightControl_Update(void)
{
    if (!g_mpu_ok) return;

    // 读取MPU6050
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (MPU6050_ReadAccel(&accel_x, &accel_y, &accel_z) != HAL_OK ||
        MPU6050_ReadGyro(&gyro_x, &gyro_y, &gyro_z) != HAL_OK)
    {
        g_mpu_read_ok = 0;
        return;
    }
    g_mpu_read_ok = 1;

    // 计算角度（使用固定dt=2.5ms，因为定时器频率为400Hz）
    Angle_t local_angle;
    MPU6050_ComputeAngles(accel_x, accel_y, accel_z,
                           gyro_x, gyro_y, gyro_z,
                           &local_angle, FLIGHT_CONTROL_DT);
    g_angle = local_angle;

    // 初始化阶段：只读角度，不控制电机
    if (g_init_phase) return;

    // 已降落：关闭电机
    if (g_has_landed) {
        Motor_SetAll(1000, 1000, 1000, 1000);
        g_motor1 = g_motor2 = g_motor3 = g_motor4 = 1000;
        return;
    }

    // 未解锁：电机最低油门
    if (!g_rc_armed) {
        Motor_SetAll(1000, 1000, 1000, 1000);
        g_motor1 = g_motor2 = g_motor3 = g_motor4 = 1000;
        return;
    }

    // 应用角度死区
    if (fabsf(local_angle.pitch) < ANGLE_DEADZONE) local_angle.pitch = 0.0f;
    if (fabsf(local_angle.roll) < ANGLE_DEADZONE) local_angle.roll = 0.0f;

    // 角度模式PID：误差 = 目标角度 - 实际角度
    float pitch_error = g_rc_pitch_target - local_angle.pitch;
    float roll_error  = g_rc_roll_target  - local_angle.roll;

    // 比例控制
    int16_t pitch_adj = (int16_t)(pitch_error * 2.0f);
    int16_t roll_adj  = (int16_t)(roll_error * 2.0f);
    if (pitch_adj > 500)  pitch_adj = 500;
    if (pitch_adj < -500) pitch_adj = -500;
    if (roll_adj > 500)   roll_adj = 500;
    if (roll_adj < -500)  roll_adj = -500;

    g_pitch_adjust = pitch_adj;
    g_roll_adjust = roll_adj;

    // 读取基础油门
    uint16_t base = g_base_throttle;

    // 混控输出
    uint16_t m1 = base + pitch_adj - roll_adj;
    uint16_t m2 = base + pitch_adj + roll_adj;
    uint16_t m3 = base - pitch_adj - roll_adj;
    uint16_t m4 = base - pitch_adj + roll_adj;

    // 限幅
    if (m1 < 1000) m1 = 1000; if (m1 > 2000) m1 = 2000;
    if (m2 < 1000) m2 = 1000; if (m2 > 2000) m2 = 2000;
    if (m3 < 1000) m3 = 1000; if (m3 > 2000) m3 = 2000;
    if (m4 < 1000) m4 = 1000; if (m4 > 2000) m4 = 2000;

    // 设置电机PWM
    Motor_SetAll(m1, m2, m3, m4);

    // 保存到共享变量
    g_motor1 = m1;
    g_motor2 = m2;
    g_motor3 = m3;
    g_motor4 = m4;
}

/**
 * @brief UART接收完成回调（DMA传输完成时调用）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        IBUS_RxCpltCallback();
    }
}

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
