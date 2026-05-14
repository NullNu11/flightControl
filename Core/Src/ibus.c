/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ibus.c
  * @brief   FlySky i-BUS protocol parser
  *          接收机: FS-iA10B, 连接 PA3 (USART2_RX)
  *
  * V5: HAL IT逐字节接收 + 环形缓冲区 + 深度DR诊断
  ******************************************************************************
  */
/* USER CODE END Header */
#include "ibus.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define USE_INTERRUPT_RX     1

#if !USE_INTERRUPT_RX
DMA_HandleTypeDef hdma_usart2_rx;
#endif

/* 环形队列缓冲区 */
#define IBUS_RING_SIZE      512
static volatile uint8_t ibus_ring[IBUS_RING_SIZE];
static volatile uint16_t ibus_ring_head = 0;
static volatile uint16_t ibus_ring_tail = 0;

/* 帧解析临时缓冲区 */
static uint8_t ibus_frame_buf[IBUS_FRAME_SIZE];

/* 通道数据 */
static volatile uint16_t ibus_channels[IBUS_CHANNELS];
static volatile uint8_t  ibus_frame_valid = 0;
static volatile uint32_t ibus_last_frame_time = 0;

/* 调试统计 */
static volatile uint32_t dbg_rx_bytes    = 0;
static volatile uint32_t dbg_idle_count  = 0;
static volatile uint32_t dbg_parse_ok    = 0;
static volatile uint32_t dbg_header_fail = 0;
static volatile uint32_t dbg_checksum_fail = 0;
static volatile uint32_t dbg_overflow     = 0;
static volatile uint32_t dbg_rxne_poll_ok= 0;   /* 轮询DR读取成功次数 */

/* 最近收到的原始字节（回调中记录）*/
static volatile uint8_t dbg_last_bytes[16] = {0};
static volatile uint8_t dbg_last_idx = 0;

/* 单字节接收缓冲（HAL IT模式）*/
static volatile uint8_t ibus_rx_byte = 0;


/**
  * @brief 初始化i-BUS接收
  */
void IBUS_Init(void)
{
    memset((void*)ibus_ring, 0, sizeof(ibus_ring));
    memset(ibus_frame_buf, 0, sizeof(ibus_frame_buf));
    memset((void*)dbg_last_bytes, 0, sizeof(dbg_last_bytes));
    ibus_ring_head = 0;
    ibus_ring_tail = 0;
    for (uint8_t i = 0; i < IBUS_CHANNELS; i++) ibus_channels[i] = IBUS_CH_CENTER;
    ibus_frame_valid = 0;

#if USE_INTERRUPT_RX
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* 启动HAL IT模式：每次收1字节触发回调 */
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&ibus_rx_byte, 1);

#else
    /* DMA方式（暂未使用）*/
#endif
}


#if USE_INTERRUPT_RX

/**
  * @brief HAL UART RxCpltCallback → 被main.c的HAL_UART_RxCpltCallback转发
  */
void IBUS_RxCpltCallback(void)
{
    uint8_t byte = ibus_rx_byte;
    dbg_rx_bytes++;

    dbg_last_bytes[dbg_last_idx++ & 0x0F] = byte;

    uint16_t next_head = (ibus_ring_head + 1) % IBUS_RING_SIZE;
    if (next_head != ibus_ring_tail)
    {
        ibus_ring[ibus_ring_head] = byte;
        ibus_ring_head = next_head;
    }
    else
    {
        dbg_overflow++;
    }

    /* 重启下一次接收 */
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&ibus_rx_byte, 1);
}

#endif


void IBUS_ProcessIdle(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        dbg_idle_count++;
    }
}


static uint16_t IBUS_Available(void)
{
    return (ibus_ring_head - ibus_ring_tail + IBUS_RING_SIZE) % IBUS_RING_SIZE;
}

static uint8_t IBUS_ReadByte(void)
{
    if (ibus_ring_tail == ibus_ring_head) return 0;
    uint8_t byte = ibus_ring[ibus_ring_tail];
    ibus_ring_tail = (ibus_ring_tail + 1) % IBUS_RING_SIZE;
    return byte;
}

static uint8_t IBUS_ValidateChecksum(const uint8_t *data)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) sum += data[i];
    uint16_t checksum = (uint16_t)(data[30] | (data[31] << 8));
    return ((uint16_t)(0xFFFF - sum) == checksum);
}


void IBUS_Update(void)
{
    if (IBUS_Available() < IBUS_FRAME_SIZE) return;

    uint8_t tmp_buf[256];
    uint16_t avail = IBUS_Available();
    if (avail > sizeof(tmp_buf)) avail = sizeof(tmp_buf);
    uint16_t i;
    for (i = 0; i < avail; i++)
        tmp_buf[i] = IBUS_ReadByte();

    int16_t max_offset = (int16_t)avail - (int16_t)IBUS_FRAME_SIZE;
    int16_t found = -1;

    for (int16_t offset = 0; offset <= max_offset; offset++)
    {
        if (tmp_buf[offset] == 0x20 && tmp_buf[offset + 1] == 0x40)
        {
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


uint16_t IBUS_GetChannel(uint8_t ch)
{
    if (ch >= IBUS_CHANNELS) return IBUS_CH_CENTER;
    return ibus_channels[ch];
}

uint8_t IBUS_IsConnected(void)
{
    if (!ibus_frame_valid) return 0;
    return (HAL_GetTick() - ibus_last_frame_time) < IBUS_TIMEOUT_MS;
}


/**
 * @brief 精简调试输出（减少阻塞时间，避免影响i-BUS实时性）
 */
void IBUS_DebugPrint(void)
{
    char buf[200];
    extern UART_HandleTypeDef huart1;

    sprintf(buf,
            "i-BUS: RX=%lu OK=%lu HERR=%lu CERR=%lu OVF=%lu V=%u\r\n",
            dbg_rx_bytes, dbg_parse_ok, dbg_header_fail, dbg_checksum_fail,
            dbg_overflow, ibus_frame_valid);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    /* 最近收到的16字节（轻量）*/
    sprintf(buf, "LAST: ");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    for (int i = 0; i < 16; i++)
    {
        sprintf(buf, "%02X ", dbg_last_bytes[(dbg_last_idx - 16 + i) & 0x0F]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    }
    sprintf(buf, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);

    /* 有效帧时打印通道 */
    if (ibus_frame_valid)
    {
        sprintf(buf, "CH: R=%d P=%d T=%d Y=%d A=%d\r\n",
                ibus_channels[0], ibus_channels[1], ibus_channels[2],
                ibus_channels[3], ibus_channels[4]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 50);
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
}
