/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ibus.c
  * @brief   FlySky i-BUS protocol parser
  *          USART2 DMA接收 + IDLE中断帧对齐
  *          接收机: FS-iA10B, 连接 PA3 (USART2_RX)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "ibus.h"
#include "usart.h"
#include <string.h>

/* DMA句柄: USART2_RX 使用 DMA1 Stream5 Channel4 */
DMA_HandleTypeDef hdma_usart2_rx;

/* 双缓冲区（交替使用） */
#define IBUS_BUF_SIZE     64      // 稍大一些防止溢出
static uint8_t ibus_buf_a[IBUS_BUF_SIZE];
static uint8_t ibus_buf_b[IBUS_BUF_SIZE];
static uint8_t *ibus_active_buf = ibus_buf_a;
static volatile uint16_t ibus_received_len = 0;

/* 帧解析缓冲区（从双缓冲区复制过来解析）*/
static uint8_t ibus_rx_buffer[IBUS_FRAME_SIZE];

/* 通道数据 */
static volatile uint16_t ibus_channels[IBUS_CHANNELS];
static volatile uint8_t  ibus_frame_valid = 0;
static volatile uint32_t ibus_last_frame_time = 0;

/* 调试计数器 */
static volatile uint32_t dbg_idle_count = 0;
static volatile uint32_t dbg_dma_count  = 0;
static volatile uint32_t dbg_checksum_fail = 0;
static volatile uint32_t dbg_header_fail = 0;

/**
 * @brief 初始化i-BUS接收
 *        配置DMA + IDLE中断，启动接收
 */
void IBUS_Init(void)
{
    /* 使能DMA1时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 配置 DMA1 Stream5 Channel4 (USART2_RX) - 循环模式 */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel           = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction         = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc         = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc            = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment  = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode              = DMA_CIRCULAR;      // 循环模式，不会停止
    hdma_usart2_rx.Init.Priority          = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode          = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* 关联DMA到USART2 */
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* 配置NVIC */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* 使能IDLE中断（帧边界检测） */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* 启动循环DMA接收（使用buf_a作为初始缓冲区）*/
    ibus_active_buf = ibus_buf_a;
    memset(ibus_buf_a, 0, IBUS_BUF_SIZE);
    memset(ibus_buf_b, 0, IBUS_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart2, ibus_active_buf, IBUS_BUF_SIZE);
}

/**
 * @brief 校验i-BUS帧校验和
 *        checksum = 0xFFFF - sum(bytes[0..29])
 */
static uint8_t IBUS_ValidateChecksum(const uint8_t *data)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
        sum += data[i];
    }
    uint16_t checksum = (uint16_t)(data[30] | (data[31] << 8));
    return ((uint16_t)(0xFFFF - sum) == checksum);
}

/**
 * @brief 解析i-BUS帧
 *        帧格式: [0x20] [0x40] [CH1L CH1H] ... [CH14L CH14H] [CKL CKH]
 */
static void IBUS_ParseFrame(void)
{
    const uint8_t *data = ibus_rx_buffer;

    /* 验证帧头 */
    if (data[0] != 0x20 || data[1] != 0x40) {
        dbg_header_fail++;
        return;
    }

    /* 验证校验和 */
    if (!IBUS_ValidateChecksum(data)) {
        dbg_checksum_fail++;
        return;
    }

    /* 提取14个通道值（小端序，11-bit有效） */
    for (uint8_t i = 0; i < IBUS_CHANNELS; i++)
    {
        uint16_t val = (uint16_t)(data[2 + i * 2] | (data[3 + i * 2] << 8));
        if (val < IBUS_CH_MIN) val = IBUS_CH_MIN;
        if (val > IBUS_CH_MAX) val = IBUS_CH_MAX;
        ibus_channels[i] = val;
    }

    ibus_frame_valid = 1;
    ibus_last_frame_time = HAL_GetTick();
}

/**
 * @brief 获取指定通道值
 * @param ch 通道号 (0-13)
 * @return 通道值 (1000-2000), 无效通道返回1500
 */
uint16_t IBUS_GetChannel(uint8_t ch)
{
    if (ch >= IBUS_CHANNELS) return IBUS_CH_CENTER;
    return ibus_channels[ch];
}

/**
 * @brief 检查接收机是否连接
 * @return 1=连接正常, 0=超时断开
 */
uint8_t IBUS_IsConnected(void)
{
    if (!ibus_frame_valid) return 0;
    return (HAL_GetTick() - ibus_last_frame_time) < IBUS_TIMEOUT_MS;
}

/**
 * @brief IDLE中断处理（循环DMA + 双缓冲帧对齐）
 *        IDLE = 帧间空闲，此时切换缓冲区并解析已收到的数据
 */
void IBUS_ProcessIdle(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        dbg_idle_count++;

        /* 计算当前已接收的字节数（循环DMA总长度 - 剩余计数）*/
        uint16_t received = (uint16_t)(IBUS_BUF_SIZE - __HAL_DMA_GET_COUNTER(hdma_usart2_rx.Instance));

        if (received >= IBUS_FRAME_SIZE)
        {
            /* 已收到完整帧，复制数据并解析 */
            uint8_t *process_buf;
            uint8_t *next_buf;

            if (ibus_active_buf == ibus_buf_a) {
                process_buf = ibus_buf_a;
                next_buf    = ibus_buf_b;
            } else {
                process_buf = ibus_buf_b;
                next_buf    = ibus_buf_a;
            }

            /* 复制原始数据到静态缓冲区供解析使用 */
            memcpy(ibus_rx_buffer, process_buf, IBUS_FRAME_SIZE);

            /* 切换到另一个缓冲区，清零后重启DMA */
            memset(next_buf, 0, IBUS_BUF_SIZE);
            ibus_active_buf = next_buf;

            /* 停止并重新启动DMA（循环模式下需要这样切换缓冲区）*/
            HAL_UART_DMAStop(&huart2);
            HAL_UART_Receive_DMA(&huart2, ibus_active_buf, IBUS_BUF_SIZE);

            /* 解析帧 */
            dbg_dma_count++;
            IBUS_ParseFrame();
        }
        /* received < 32: 数据还不够，等待更多数据，不做任何操作 */
    }
}

/**
 * @brief DMA接收完成回调
 *        由 HAL_UART_RxCpltCallback 调用
 */
/**
 * @brief DMA接收完成回调（循环DMA缓冲区满时触发）
 *        正常情况下帧在IDLE中断中处理，此处作为安全兜底
 */
void IBUS_RxCpltCallback(void)
{
    /* 循环模式下DMA会自动继续，无需重启 */
    dbg_dma_count++;
}

/**
 * @brief 打印i-BUS调试信息（用于排查连接问题）
 */
void IBUS_DebugPrint(void)
{
    char buf[120];
    extern UART_HandleTypeDef huart1;
    sprintf(buf, "i-BUS DBG: IDLE=%lu DMA=%lu HDR_ERR=%lu CHK_ERR=%lu VALID=%u\r\n",
            dbg_idle_count, dbg_dma_count, dbg_header_fail, dbg_checksum_fail, ibus_frame_valid);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    // 打印原始帧数据（前8字节）
    if (dbg_dma_count > 0)
    {
        sprintf(buf, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                ibus_rx_buffer[0], ibus_rx_buffer[1], ibus_rx_buffer[2], ibus_rx_buffer[3],
                ibus_rx_buffer[4], ibus_rx_buffer[5], ibus_rx_buffer[6], ibus_rx_buffer[7]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

        // 如果有有效帧，打印通道值
        if (ibus_frame_valid)
        {
            sprintf(buf, "CH: R=%d P=%d T=%d Y=%d A=%d\r\n",
                    ibus_channels[0], ibus_channels[1], ibus_channels[2],
                    ibus_channels[3], ibus_channels[4]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
        }
    }
}
