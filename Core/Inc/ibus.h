/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ibus.h
  * @brief   FlySky i-BUS protocol parser (DMA + IDLE interrupt)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __IBUS_H__
#define __IBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* i-BUS协议参数 */
#define IBUS_FRAME_SIZE     32      // 帧长度（字节）
#define IBUS_CHANNELS       14      // 通道数
#define IBUS_TIMEOUT_MS     500     // 连接超时
#define USE_INTERRUPT_RX    1       // 1=中断逐字节接收, 0=DMA接收

/* i-BUS通道值范围 */
#define IBUS_CH_MIN         1000
#define IBUS_CH_MAX         2000
#define IBUS_CH_CENTER      1500

/* 通道映射（Mode 2） */
#define IBUS_CH_ROLL        0       // CH1: 副翼 Aileron
#define IBUS_CH_PITCH       1       // CH2: 升降 Elevator
#define IBUS_CH_THROTTLE    2       // CH3: 油门 Throttle
#define IBUS_CH_YAW         3       // CH4: 方向 Rudder
#define IBUS_CH_ARM         4       // CH5: 解锁开关

/* 摇杆满偏对应的最大目标角度（度） */
#define RC_MAX_ANGLE        30.0f

void IBUS_Init(void);
void IBUS_Update(void);                    // 主循环调用：从环形缓冲区解析帧
uint16_t IBUS_GetChannel(uint8_t ch);
uint8_t IBUS_IsConnected(void);
void IBUS_ProcessIdle(void);               // USART2_IRQHandler中调用
void IBUS_OnRxByte(uint8_t byte);          // RXNE中断回调：写入环形缓冲区
void IBUS_RxCpltCallback(void);            // DMA TC回调
void IBUS_DebugPrint(void);                // 调试输出

#ifdef __cplusplus
}
#endif

#endif /* __IBUS_H__ */
