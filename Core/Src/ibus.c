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

/* DMA句柄: USART2_RX 使用 DMA1 Stream5 Channel4 */
DMA_HandleTypeDef hdma_usart2_rx;

/* 接收缓冲区 */
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

    /* 配置 DMA1 Stream5 Channel4 (USART2_RX) */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel           = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction         = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc         = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc            = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment  = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode              = DMA_NORMAL;
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
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* 使能IDLE中断（帧边界检测） */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* 启动DMA接收 */
    HAL_UART_Receive_DMA(&huart2, ibus_rx_buffer, IBUS_FRAME_SIZE);
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
 * @brief IDLE中断处理
 *        帧边界检测：IDLE时DMA未完成说明帧未对齐，重启DMA
 *        由 USART2_IRQHandler 调用
 */
void IBUS_ProcessIdle(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        dbg_idle_count++;

        /* DMA仍在传输 = 起始位置不在帧头，重启对齐 */
        if (huart2.RxXferCount != 0)
        {
            HAL_UART_DMAStop(&huart2);
            HAL_UART_Receive_DMA(&huart2, ibus_rx_buffer, IBUS_FRAME_SIZE);
        }
    }
}

/**
 * @brief DMA接收完成回调
 *        由 HAL_UART_RxCpltCallback 调用
 */
void IBUS_RxCpltCallback(void)
{
    dbg_dma_count++;
    IBUS_ParseFrame();
    /* 重启DMA接收下一帧 */
    HAL_UART_Receive_DMA(&huart2, ibus_rx_buffer, IBUS_FRAME_SIZE);
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
