/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ibus.c
  * @brief   FlySky i-BUS protocol parser
  *          接收机: FS-iA10B, 连接 PA3 (USART2_RX)
  *
  * V4: 逐字节中断接收 + 帧头搜索（用于诊断和稳定工作）
  ******************************************************************************
  */
/* USER CODE END Header */
#include "ibus.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* ====== 接收方式选择 ====== */
#define USE_INTERRUPT_RX     1    // 1=中断逐字节(可靠), 0=DMA(待修复)

#if !USE_INTERRUPT_RX
DMA_HandleTypeDef hdma_usart2_rx;
#endif

/* 接收缓冲区 - 环形队列 */
#define IBUS_RING_SIZE      128
static volatile uint8_t ibus_ring[IBUS_RING_SIZE];
static volatile uint16_t ibus_ring_head = 0;    // 写入位置（ISR）
static volatile uint16_t ibus_ring_tail = 0;    // 读取位置（主循环）

/* 帧解析缓冲区 */
static uint8_t ibus_frame_buf[IBUS_FRAME_SIZE];

/* 通道数据 */
static volatile uint16_t ibus_channels[IBUS_CHANNELS];
static volatile uint8_t  ibus_frame_valid = 0;
static volatile uint32_t ibus_last_frame_time = 0;

/* 调试统计 */
static volatile uint32_t dbg_rx_bytes    = 0;    // 总接收字节数
static volatile uint32_t dbg_idle_count  = 0;    // IDLE中断次数
static volatile uint32_t dbg_parse_ok    = 0;    // 成功解析帧数
static volatile uint32_t dbg_header_fail = 0;    // 帧头错误次数
static volatile uint32_t dbg_checksum_fail = 0;  // 校验错误次数
static volatile uint32_t dbg_overflow     = 0;    // 环形缓冲区溢出次数

/* 最近收到的原始字节（用于调试）*/
static volatile uint8_t dbg_last_bytes[16] = {0};
static volatile uint8_t dbg_last_idx = 0;


/**
 * @brief 初始化i-BUS接收
 */
void IBUS_Init(void)
{
    /* 清零所有缓冲区和变量 */
    memset((void*)ibus_ring, 0, sizeof(ibus_ring));
    memset(ibus_frame_buf, 0, sizeof(ibus_frame_buf));
    memset((void*)dbg_last_bytes, 0, sizeof(dbg_last_bytes));
    ibus_ring_head = 0;
    ibus_ring_tail = 0;
    for (uint8_t i = 0; i < IBUS_CHANNELS; i++) ibus_channels[i] = IBUS_CH_CENTER;
    ibus_frame_valid = 0;

#if USE_INTERRUPT_RX
    /* ========== 中断接收方式 ========== */
    /* 使能USART2 RXNE中断（接收寄存器非空中断）*/
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    /* 使能IDLE中断（帧间空闲检测）*/
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* NVIC配置 */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

#else
    /* ========== DMA接收方式 ========== */
    __HAL_RCC_DMA1_CLK_ENABLE();

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
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    static uint8_t dma_buf[64];
    memset(dma_buf, 0, 64);
    HAL_UART_Receive_DMA(&huart2, dma_buf, 64);
#endif
}


#if USE_INTERRUPT_RX

/**
 * @brief 单字节接收回调（由 USART2_IRQHandler 调用）
 *        每收到一个字节就写入环形缓冲区
 *        注意：此函数在中断上下文中运行，必须快速返回
 */
void IBUS_OnRxByte(uint8_t byte)
{
    dbg_rx_bytes++;

    /* 保存最近16字节用于调试 */
    dbg_last_bytes[dbg_last_idx++ & 0x0F] = byte;

    /* 写入环形缓冲区 */
    uint16_t next_head = (ibus_ring_head + 1) % IBUS_RING_SIZE;
    if (next_head != ibus_ring_tail)
    {
        ibus_ring[ibus_ring_head] = byte;
        ibus_ring_head = next_head;
    }
    else
    {
        dbg_overflow++;  // 缓冲区满，丢掉
    }
}

#endif


/**
 * @brief IDLE中断处理（帧边界检测）
 */
void IBUS_ProcessIdle(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        dbg_idle_count++;
        /* IDLE仅作为帧结束标记，实际数据处理在主循环中完成 */
    }
}


/**
 * @brief 从环形缓冲区读取可用字节数
 */
static uint16_t IBUS_Available(void)
{
    return (ibus_ring_head - ibus_ring_tail + IBUS_RING_SIZE) % IBUS_RING_SIZE;
}

/**
 * @brief 从环形缓冲区读取一个字节
 */
static uint8_t IBUS_ReadByte(void)
{
    if (ibus_ring_tail == ibus_ring_head) return 0;
    uint8_t byte = ibus_ring[ibus_ring_tail];
    ibus_ring_tail = (ibus_ring_tail + 1) % IBUS_RING_SIZE;
    return byte;
}


/**
 * @brief 校验i-BUS帧校验和
 */
static uint8_t IBUS_ValidateChecksum(const uint8_t *data)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++)
        sum += data[i];

    uint16_t checksum = (uint16_t)(data[30] | (data[31] << 8));
    return ((uint16_t)(0xFFFF - sum) == checksum);
}


/**
 * @brief 尝试在环形缓冲区中查找并解析一帧i-BUS数据
 *        在主循环中调用（非中断上下文）
 */
void IBUS_Update(void)
{
    /* 至少需要32字节才能尝试解析 */
    if (IBUS_Available() < IBUS_FRAME_SIZE) return;

    /* 搜索帧头 0x20 0x40 */
    /* 策略：先把足够多的字节读到临时缓冲区，然后搜索 */

    /* 读取最多一帧+偏移量的数据 */
    uint8_t tmp_buf[48];
    uint16_t avail = IBUS_Available();
    if (avail > sizeof(tmp_buf)) avail = sizeof(tmp_buf);

    uint16_t i;
    for (i = 0; i < avail; i++)
        tmp_buf[i] = IBUS_ReadByte();

    /* 在临时缓冲区中搜索帧头 */
    int16_t max_offset = (int16_t)avail - (int16_t)IBUS_FRAME_SIZE;
    int16_t found = -1;

    for (int16_t offset = 0; offset <= max_offset; offset++)
    {
        if (tmp_buf[offset] == 0x20 && tmp_buf[offset + 1] == 0x40)
        {
            /* 候选帧头找到，验证校验和 */
            const uint8_t *frame = tmp_buf + offset;
            if (IBUS_ValidateChecksum(frame))
            {
                found = offset;
                break;
            }
            else
            {
                dbg_checksum_fail++;
            }
        }
    }

    if (found >= 0)
    {
        /* 有效帧！提取14个通道 */
        const uint8_t *frame = tmp_buf + found;
        for (i = 0; i < IBUS_CHANNELS; i++)
        {
            uint16_t val = (uint16_t)(frame[2 + i*2] | (frame[3 + i*2] << 8));
            if (val < IBUS_CH_MIN) val = IBUS_CH_MIN;
            if (val > IBUS_CH_MAX) val = IBUS_CH_MAX;
            ibus_channels[i] = val;
        }

        ibus_frame_valid = 1;
        ibus_last_frame_time = HAL_GetTick();
        dbg_parse_ok++;
    }
    else
    {
        dbg_header_fail++;
    }
}


/**
 * @brief 获取指定通道值
 */
uint16_t IBUS_GetChannel(uint8_t ch)
{
    if (ch >= IBUS_CHANNELS) return IBUS_CH_CENTER;
    return ibus_channels[ch];
}

/**
 * @brief 检查接收机是否连接
 */
uint8_t IBUS_IsConnected(void)
{
    if (!ibus_frame_valid) return 0;
    return (HAL_GetTick() - ibus_last_frame_time) < IBUS_TIMEOUT_MS;
}

/**
 * @brief DMA TC 回调（DMA模式下使用）
 */
void IBUS_RxCpltCallback(void)
{
    /* 不使用 */
}

/**
 * @brief 打印详细调试信息
 */
void IBUS_DebugPrint(void)
{
    char buf[200];
    extern UART_HandleTypeDef huart1;

    sprintf(buf,
            "--- i-BUS DEBUG ---\r\n"
            "RX_BYTES=%lu  IDLE=%lu  AVAIL=%u\r\n"
            "PARSE_OK=%lu  H_ERR=%lu  C_ERR=%lu  OVF=%lu\r\n"
            "VALID=%u  LAST_MS=%lu\r\n",
            dbg_rx_bytes, dbg_idle_count, (unsigned)IBUS_Available(),
            dbg_parse_ok, dbg_header_fail, dbg_checksum_fail, dbg_overflow,
            ibus_frame_valid, ibus_last_frame_time ? (HAL_GetTick() - ibus_last_frame_time) : 0);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    /* 最近收到的16个原始字节 */
    sprintf(buf, "LAST16: ");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    for (int i = 0; i < 16; i++)
    {
        sprintf(buf, "%02X ", dbg_last_bytes[(dbg_last_idx - 16 + i) & 0x0F]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    }
    sprintf(buf, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    /* USART2 状态标志检查 */
    uint32_t sr = huart2.Instance->SR;
    sprintf(buf, "USART_SR: ORE=%lu FE=%lu PE=%lu NF=%lu RXNE=%lu IDLE=%lu\r\n",
            (sr >> 3) & 1, (sr >> 1) & 1, (sr >> 0) & 1,
            (sr >> 2) & 1, (sr >> 5) & 1, (sr >> 4) & 1);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    /* 如果有效，打印通道值 */
    if (ibus_frame_valid)
    {
        sprintf(buf, "CH: R=%d P=%d T=%d Y=%d A=%d\r\n",
                ibus_channels[0], ibus_channels[1], ibus_channels[2],
                ibus_channels[3], ibus_channels[4]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    }

    /* 清除可能的USART错误标志 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
}
