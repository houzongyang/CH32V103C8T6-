#include "rplidar_360.h"
#include <stdint.h>
#include <string.h>

/**
 * @file    rplidar_360_driver.c   (假设文件名)
 * @brief   RPLIDAR 360 激光雷达驱动（STM32 + USART1 + DMA）
 * @note    实现数据接收、协议解析、360度实时距离分箱
 */

/*=============================================================================
 * 配置宏
 *============================================================================*/

#ifndef LIDAR_DMA_RX_LEN
#define LIDAR_DMA_RX_LEN                 128u   /*!< DMA接收缓冲区长度（字节）*/
#endif

#ifndef LIDAR_RING_LEN
#define LIDAR_RING_LEN                   1024u  /*!< 软件环形缓冲区大小（字节）*/
#endif

#ifndef LIDAR_USART_RX_CH
#define LIDAR_USART_RX_CH                DMA1_Channel5  /*!< 使用的DMA通道 */
#endif

#define RPLIDAR360_ANGLE_Q6_MAX          23040u /*!< 角度最大值（Q6格式，360°*64）*/

#define RPLIDAR_SCAN_DESC_LEN            7u     /*!< 扫描描述头长度 */
#define RPLIDAR_POINT_RAW_LEN            5u     /*!< 单个点原始数据长度 */
#define RPLIDAR_USABLE_FRONT_MIN_BINS    24u
#define RPLIDAR_USABLE_RAW_MIN_POINTS    48u
#define RPLIDAR_USABLE_FRONT_LEFT_DEG   -105
#define RPLIDAR_USABLE_FRONT_RIGHT_DEG   105
#define RPLIDAR_FRONT_OFFSET_DEG         0

#define RPLIDAR_STATE_WAIT_DESC          0u     /*!< 状态：等待描述头 */
#define RPLIDAR_STATE_RECV_POINT         1u     /*!< 状态：接收点数据 */

#define RPLIDAR_PARSE_OK                 0u     /*!< 解析成功 */
#define RPLIDAR_PARSE_BAD_POINT          1u     /*!< 无效点 */

/*=============================================================================
 * 内部数据结构和函数指针
 *============================================================================*/

/**
 * @brief 发送字节流函数指针类型（由底层实现）
 */
typedef void (*RPLIDAR360_SendFunc_t)(const uint8_t *data, uint16_t len);

/**
 * @brief 延时函数指针类型（由底层实现）
 */
typedef void (*RPLIDAR360_DelayMsFunc_t)(uint32_t ms);

/**
 * @brief 实时分箱数据结构（存储当前扫描周期内每个角度的最新距离）
 */
typedef struct
{
    RPLIDAR360_Frame_t frame;   /*!< 当前帧数据（360个bin） */
    uint8_t started;            /*!< 是否已开始接收 */
    uint32_t round_seq_next;    /*!< 下一帧序号 */
} RPLIDAR360_Box_t;

/**
 * @brief 驱动核心控制块
 */
typedef struct
{
    uint8_t *ring_buf;                 /*!< 环形缓冲区指针 */
    uint16_t ring_size;                /*!< 环形缓冲区大小 */
    volatile uint16_t ring_w;          /*!< 写指针 */
    volatile uint16_t ring_r;          /*!< 读指针 */

    uint8_t state;                     /*!< 协议解析状态 */
    uint8_t desc_index;                /*!< 描述头匹配索引 */
    uint8_t point_len;                 /*!< 当前点数据已接收长度 */
    uint8_t point_buf[RPLIDAR_POINT_RAW_LEN]; /*!< 点数据临时缓存 */

    RPLIDAR360_SendFunc_t send;        /*!< 发送函数 */
    RPLIDAR360_DelayMsFunc_t delay_ms; /*!< 延时函数 */

    RPLIDAR360_Box_t box;              /*!< 分箱数据 */
} RPLIDAR360_Driver_t;

/**
 * @brief DMA双缓冲控制块
 */
typedef struct
{
    uint8_t dma_use_buffer;            /*!< 当前使用的缓冲区索引（0或1） */
    uint8_t rx_buf[2][LIDAR_DMA_RX_LEN]; /*!< 双缓冲区 */
} RPLIDAR360_DmaCtrl_t;

/*=============================================================================
 * 全局变量
 *============================================================================*/

/* RPLIDAR标准扫描描述头：7字节，固定值 */
static const uint8_t s_rplidar_scan_desc[RPLIDAR_SCAN_DESC_LEN] =
{
    0xA5u, 0x5Au, 0x05u, 0x00u, 0x00u, 0x40u, 0x81u
};

static RPLIDAR360_Driver_t g_rplidar;          /*!< 全局驱动实例 */
static uint8_t g_rplidar_ring_buf[LIDAR_RING_LEN]; /*!< 软件环形缓冲区 */

static RPLIDAR360_DmaCtrl_t g_lidar_dma;       /*!< DMA控制块 */

static uint16_t RPLIDAR360_SignedDegToIndex(int16_t signed_deg)
{
    int16_t lidar_deg = (int16_t)(signed_deg + RPLIDAR_FRONT_OFFSET_DEG);

    while (lidar_deg >= 360)
    {
        lidar_deg -= 360;
    }

    while (lidar_deg < 0)
    {
        lidar_deg += 360;
    }

    return (uint16_t)lidar_deg;
}

static uint16_t RPLIDAR360_CountFrontValidBins(const RPLIDAR360_Frame_t *frame)
{
    int16_t deg;
    uint16_t count = 0u;

    if (frame == 0)
    {
        return 0u;
    }

    for (deg = RPLIDAR_USABLE_FRONT_LEFT_DEG;
         deg <= RPLIDAR_USABLE_FRONT_RIGHT_DEG;
         deg++)
    {
        uint16_t idx = RPLIDAR360_SignedDegToIndex(deg);

        if ((idx < RPLIDAR360_BIN_COUNT) &&
            (frame->bin[idx].valid != 0u))
        {
            count++;
        }
    }

    return count;
}

static uint8_t RPLIDAR360_IsFrameUsableForDrive(const RPLIDAR360_Frame_t *frame)
{
    if (frame == 0)
    {
        return 0u;
    }

    if ((frame->raw_point_count < RPLIDAR_USABLE_RAW_MIN_POINTS) ||
        (frame->filled_count < RPLIDAR_USABLE_FRONT_MIN_BINS))
    {
        return 0u;
    }

    if (RPLIDAR360_CountFrontValidBins(frame) < RPLIDAR_USABLE_FRONT_MIN_BINS)
    {
        return 0u;
    }

    return 1u;
}


/**
 * @brief 清空分箱数据（开始新一轮扫描）
 * @param box 分箱结构指针
 */
static void RPLIDAR360_BoxClearData(RPLIDAR360_Box_t *box)
{
    if (box == 0)
    {
        return;
    }

    RPLIDAR360_Frame_t *active = &box->frame;
    memset(active, 0, sizeof(RPLIDAR360_Frame_t));
    active->round_seq = box->round_seq_next++;
}

/*=============================================================================
 * 实时分箱处理
 *============================================================================*/

/**
 * @brief 初始化分箱结构
 * @param box 分箱结构指针
 */
static void RPLIDAR360_BoxInit(RPLIDAR360_Box_t *box)
{
    if (box == 0) return;
    memset(box, 0, sizeof(RPLIDAR360_Box_t));
    box->round_seq_next = 1u;
}

/**
 * @brief 输入一个原始点数据到分箱（实时更新每个角度最新值）
 * @param box         分箱结构指针
 * @param angle_q6    角度（Q6格式, 0~23040）
 * @param distance_q2 距离（Q2格式，单位mm/4）
 * @param quality     信号质量
 * @param start_flag  新一周期的起始标志（S位）
 * @return 1表示新扫描周期开始，0表示未开始
 */
static uint8_t RPLIDAR360_BoxInputRaw(RPLIDAR360_Box_t *box,
                                      uint16_t angle_q6,
                                      uint16_t distance_q2,
                                      uint8_t quality,
                                      uint8_t start_flag)
{
    uint16_t degree;
    uint16_t distance_mm;
    RPLIDAR360_Frame_t *active;
    uint8_t new_round = 0u;

    if (box == 0) return 0u;

    active = &box->frame;

    /* 如果还未开始，则初始化新周期 */
    if (box->started == 0u)
    {
        box->started = 1u;
        RPLIDAR360_BoxClearData(box);
        active->round_seq = box->round_seq_next++;
        new_round = 1u;
    }
    else if (start_flag != 0u)   /* 收到S=1，表示新一圈开始 */
    {
        /* 清空上一圈数据，开始新的一圈 */
        memset(active, 0, sizeof(RPLIDAR360_Frame_t));
        active->round_seq = box->round_seq_next++;
        // active->round_seq = box->round_seq_next++;
        // active->raw_point_count = 0u;
        new_round = 1u;
    }

    /* 计算整数角度（0~359） */
    degree = (uint16_t)(angle_q6 >> 6);
    if (degree >= RPLIDAR360_BIN_COUNT)
    {
        return new_round;
    }

    /* 无效距离则跳过 */
    if (distance_q2 == 0u || (distance_mm = (uint16_t)(distance_q2 >> 2)) == 0u)
    {
        return new_round;
    }

    active->raw_point_count++;

    /* 更新该角度bin */
    if (active->bin[degree].valid == 0u)
    {
        active->filled_count++;          /* 首次填充此角度 */
    }
    active->bin[degree].distance_mm = distance_mm;
    active->bin[degree].quality = quality;
    active->bin[degree].valid = 1u;

    return new_round;
}

/*=============================================================================
 * 环形缓冲区操作
 *============================================================================*/

/**
 * @brief 清空环形缓冲区
 */
static void RPLIDAR_RingClear(RPLIDAR360_Driver_t *lidar)
{
    if (lidar) lidar->ring_r = lidar->ring_w;
}

/**
 * @brief 向环形缓冲区压入一个字节
 * @return RPLIDAR_OK 成功，RPLIDAR_RING_OVERFLOW 溢出
 */
static int RPLIDAR_RingPush(RPLIDAR360_Driver_t *lidar, uint8_t data)
{
    if (lidar == 0 || lidar->ring_buf == 0 || lidar->ring_size < 2u)
        return RPLIDAR_ERR;

    uint16_t next_w = (uint16_t)(lidar->ring_w + 1u);
    if (next_w >= lidar->ring_size) next_w = 0u;

    if (next_w == lidar->ring_r)
        return RPLIDAR_RING_OVERFLOW;

    lidar->ring_buf[lidar->ring_w] = data;
    lidar->ring_w = next_w;
    return RPLIDAR_OK;
}

/**
 * @brief 从环形缓冲区弹出一个字节
 * @return RPLIDAR_OK 成功，否则失败
 */
static int RPLIDAR_RingPop(RPLIDAR360_Driver_t *lidar, uint8_t *data)
{
    if (lidar == 0 || data == 0) return RPLIDAR_ERR;
    if (lidar->ring_r == lidar->ring_w) return RPLIDAR_ERR;

    *data = lidar->ring_buf[lidar->ring_r];
    lidar->ring_r = (uint16_t)(lidar->ring_r + 1u);
    if (lidar->ring_r >= lidar->ring_size) lidar->ring_r = 0u;
    return RPLIDAR_OK;
}

/*=============================================================================
 * RPLIDAR标准SCAN协议解析
 *============================================================================*/

/**
 * @brief 检查5字节点包的有效性（校验S、S_inv、C位）
 * @param p 指向5字节数据的指针
 * @return 1有效，0无效
 */
static uint8_t RPLIDAR_IsValidPointPacket(const uint8_t *p)
{
    if (p == 0) return 0u;
    uint8_t s     = (uint8_t)(p[0] & 0x01u);
    uint8_t s_inv = (uint8_t)((p[0] >> 1) & 0x01u);
    uint8_t c     = (uint8_t)(p[1] & 0x01u);
    if ((uint8_t)(s ^ s_inv) != 1u) return 0u;
    if (c != 1u) return 0u;
    uint16_t angle_q6 = (uint16_t)(((uint16_t)(p[1] >> 1)) | ((uint16_t)p[2] << 7));
    if (angle_q6 >= RPLIDAR360_ANGLE_Q6_MAX) return 0u;
    return 1u;
}

/**
 * @brief 解析5字节点包，提取角度、距离、质量、起始标志
 * @param lidar 驱动实例
 * @param p     5字节数据
 * @return RPLIDAR_PARSE_OK 或 RPLIDAR_PARSE_BAD_POINT
 */
static uint8_t RPLIDAR_ParsePointPacket(RPLIDAR360_Driver_t *lidar, const uint8_t *p)
{
    if (lidar == 0 || p == 0) return RPLIDAR_PARSE_BAD_POINT;
    if (RPLIDAR_IsValidPointPacket(p) == 0u) return RPLIDAR_PARSE_BAD_POINT;

    uint8_t  quality    = (uint8_t)(p[0] >> 2);
    uint8_t  start_flag = (uint8_t)(p[0] & 0x01u);
    uint16_t angle_q6   = (uint16_t)(((uint16_t)(p[1] >> 1)) | ((uint16_t)p[2] << 7));
    uint16_t distance_q2 = (uint16_t)(((uint16_t)p[3]) | ((uint16_t)p[4] << 8));

    (void)RPLIDAR360_BoxInputRaw(&lidar->box, angle_q6, distance_q2, quality, start_flag);
    return RPLIDAR_PARSE_OK;
}

/**
 * @brief 字节级协议解析（状态机）
 * @param lidar 驱动实例
 * @param data  输入的字节
 * @return RPLIDAR_PARSE_OK 或 RPLIDAR_PARSE_BAD_POINT
 */
static uint8_t RPLIDAR_ParseByte(RPLIDAR360_Driver_t *lidar, uint8_t data)
{
    if (lidar == 0) return RPLIDAR_PARSE_OK;

    if (lidar->state == RPLIDAR_STATE_WAIT_DESC)
    {
        /* 匹配7字节描述头 */
        if (data == s_rplidar_scan_desc[lidar->desc_index])
        {
            lidar->desc_index++;
            if (lidar->desc_index >= RPLIDAR_SCAN_DESC_LEN)
            {
                lidar->desc_index = 0u;
                lidar->point_len = 0u;
                lidar->state = RPLIDAR_STATE_RECV_POINT;   /* 进入接收点数据状态 */
            }
        }
        else
        {
            if (data == 0xA5u)
                lidar->desc_index = 1u;   /* 可能是新头的起始 */
            else
                lidar->desc_index = 0u;
        }
        return RPLIDAR_PARSE_OK;
    }

    /* 接收点数据（5字节） */
    lidar->point_buf[lidar->point_len] = data;
    lidar->point_len++;
    if (lidar->point_len < RPLIDAR_POINT_RAW_LEN)
        return RPLIDAR_PARSE_OK;

    lidar->point_len = 0u;
    if (RPLIDAR_ParsePointPacket(lidar, lidar->point_buf) != RPLIDAR_PARSE_OK)
        return RPLIDAR_PARSE_BAD_POINT;

    return RPLIDAR_PARSE_OK;
}

/*=============================================================================
 * 驱动内部函数（协议层）
 *============================================================================*/

/**
 * @brief 初始化驱动实例
 * @param lidar      驱动实例指针
 * @param ring_buf   软件环形缓冲区
 * @param ring_size  缓冲区大小
 * @param send_func  发送函数
 * @param delay_func 延时函数
 */
static void RPLIDAR360_DriverInit(RPLIDAR360_Driver_t *lidar,
                                  uint8_t *ring_buf,
                                  uint16_t ring_size,
                                  RPLIDAR360_SendFunc_t send_func,
                                  RPLIDAR360_DelayMsFunc_t delay_func)
{
    if (lidar == 0) return;
    memset(lidar, 0, sizeof(RPLIDAR360_Driver_t));
    lidar->ring_buf = ring_buf;
    lidar->ring_size = ring_size;
    lidar->send = send_func;
    lidar->delay_ms = delay_func;
    lidar->state = RPLIDAR_STATE_WAIT_DESC;
    RPLIDAR360_BoxInit(&lidar->box);
}

/**
 * @brief 清除协议解析状态和环形缓冲区
 */
static void RPLIDAR360_ClearProtocol(RPLIDAR360_Driver_t *lidar)
{
    if (lidar == 0) return;
    lidar->ring_w = 0u;
    lidar->ring_r = 0u;
    lidar->state = RPLIDAR_STATE_WAIT_DESC;
    lidar->desc_index = 0u;
    lidar->point_len = 0u;
    RPLIDAR360_BoxInit(&lidar->box);
}

/**
 * @brief 从中断服务函数（ISR）输入原始字节流到环形缓冲区
 * @return RPLIDAR_OK 或 RPLIDAR_RING_OVERFLOW
 */
static int RPLIDAR360_InputBytesFromISR(RPLIDAR360_Driver_t *lidar,
                                        const uint8_t *data,
                                        uint16_t len)
{
    if (lidar == 0 || data == 0) return RPLIDAR_ERR;
    int ret = RPLIDAR_OK;
    for (uint16_t i = 0u; i < len; i++)
    {
        if (RPLIDAR_RingPush(lidar, data[i]) != RPLIDAR_OK)
            ret = RPLIDAR_RING_OVERFLOW;
    }
    return ret;
}

/**
 * @brief 主任务函数（轮询解析环形缓冲区中的数据）
 */
static void RPLIDAR360_Task(RPLIDAR360_Driver_t *lidar)
{
    if (lidar == 0) return;
    uint8_t data;
    while (RPLIDAR_RingPop(lidar, &data) == RPLIDAR_OK)
    {
        if (RPLIDAR_ParseByte(lidar, data) == RPLIDAR_PARSE_BAD_POINT)
        {
            /* 遇到无效点，丢弃队列中剩余数据并重新同步 */
            RPLIDAR_RingClear(lidar);
            break;
        }
    }
}

/**
 * @brief 发送停止扫描命令
 */
static int RPLIDAR360_Stop(RPLIDAR360_Driver_t *lidar)
{
    static const uint8_t cmd_stop[2] = {0xA5u, 0x25u};
    if (lidar == 0 || lidar->send == 0) return RPLIDAR_ERR;
    lidar->send(cmd_stop, 2u);
    if (lidar->delay_ms != 0) lidar->delay_ms(2u);
    return RPLIDAR_OK;
}

/**
 * @brief 发送复位命令
 */
static int RPLIDAR360_Reset(RPLIDAR360_Driver_t *lidar)
{
    static const uint8_t cmd_reset[2] = {0xA5u, 0x40u};
    if (lidar == 0 || lidar->send == 0) return RPLIDAR_ERR;
    lidar->send(cmd_reset, 2u);
    if (lidar->delay_ms != 0) lidar->delay_ms(50u);
    RPLIDAR360_ClearProtocol(lidar);
    return RPLIDAR_OK;
}

/**
 * @brief 启动扫描（先停止，清协议，再发扫描命令）
 */
static int RPLIDAR360_StartScan(RPLIDAR360_Driver_t *lidar)
{
    static const uint8_t cmd_scan[2] = {0xA5u, 0x20u};
    if (lidar == 0 || lidar->send == 0) return RPLIDAR_ERR;
    RPLIDAR360_Stop(lidar);
    RPLIDAR360_ClearProtocol(lidar);
    lidar->send(cmd_scan, 2u);
    return RPLIDAR_OK;
}

/*=============================================================================
 * 硬件层：USART1 + DMA 初始化及中断处理
 *============================================================================*/

/**
 * @brief 初始化USART1（115200，8N1，TX/ RX使能，空闲中断）
 */
static void LIDAR_USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* TX: PA9, 推挽复用 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* RX: PA10, 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);   /* 空闲中断用于DMA接收完成 */

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1u;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1u;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief 初始化DMA接收（双缓冲，循环模式？实际上是普通模式配合空闲中断切换）
 */
static void LIDAR_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(LIDAR_USART_RX_CH);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_lidar_dma.rx_buf[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = LIDAR_DMA_RX_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;          /* 非循环模式，一次传输完停止 */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(LIDAR_USART_RX_CH, &DMA_InitStructure);

    DMA_ITConfig(LIDAR_USART_RX_CH, DMA_IT_TC, ENABLE);    /* 传输完成中断 */

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1u;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0u;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    g_lidar_dma.dma_use_buffer = 0u;
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    DMA_Cmd(LIDAR_USART_RX_CH, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

/**
 * @brief 复位DMA接收（重新开始，切换到另一个缓冲区）
 */
static void LIDAR_DMA_RX_Reset(void)
{
    DMA_Cmd(LIDAR_USART_RX_CH, DISABLE);
    g_lidar_dma.dma_use_buffer = 0u;
    DMA_SetCurrDataCounter(LIDAR_USART_RX_CH, LIDAR_DMA_RX_LEN);
    LIDAR_USART_RX_CH->MADDR = (uint32_t)g_lidar_dma.rx_buf[0];
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    DMA_Cmd(LIDAR_USART_RX_CH, ENABLE);
}

/**
 * @brief 底层发送函数（阻塞轮询）
 */
static void LIDAR_SendBytes(const uint8_t *data, uint16_t len)
{
    if (data == 0) return;
    for (uint16_t i = 0u; i < len; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, data[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/**
 * @brief 包装发送函数（传递给驱动）
 */
static void RPLIDAR_PortSend(const uint8_t *data, uint16_t len)
{
    LIDAR_SendBytes(data, len);
}

/**
 * @brief 包装延时函数
 */
static void RPLIDAR_PortDelayMs(uint32_t ms)
{
    Delay_Ms(ms);   /* 需由用户提供 Delay_Ms 实现 */
}

/*=============================================================================
 * 中断服务函数
 *============================================================================*/

/**
 * @brief USART1 空闲中断处理：切换DMA缓冲区，并将已接收数据交给解析层
 */
void USART1_IRQHandler(void)
{
    volatile uint32_t temp;
    uint16_t rxlen;
    uint8_t oldbuffer;

    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(LIDAR_USART_RX_CH, DISABLE);

        rxlen = (uint16_t)(LIDAR_DMA_RX_LEN - LIDAR_USART_RX_CH->CNTR);
        oldbuffer = g_lidar_dma.dma_use_buffer;

        g_lidar_dma.dma_use_buffer ^= 1u;   /* 切换到另一个缓冲区 */

        DMA_SetCurrDataCounter(LIDAR_USART_RX_CH, LIDAR_DMA_RX_LEN);
        LIDAR_USART_RX_CH->MADDR = (uint32_t)g_lidar_dma.rx_buf[g_lidar_dma.dma_use_buffer];

        /* 清除空闲中断标志 */
        temp = USART1->STATR;
        temp = USART1->DATAR;
        (void)temp;

        DMA_Cmd(LIDAR_USART_RX_CH, ENABLE);

        if (rxlen > 0u)
        {
            (void)RPLIDAR360_InputBytesFromISR(&g_rplidar,
                                               g_lidar_dma.rx_buf[oldbuffer],
                                               rxlen);
        }
    }
}

/**
 * @brief DMA传输完成中断（实际上可能不用，空闲中断已足够；但保留用于某些情况）
 */
void DMA1_Channel5_IRQHandler(void)
{
    uint8_t oldbuffer;

    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC5);

        oldbuffer = g_lidar_dma.dma_use_buffer;
        g_lidar_dma.dma_use_buffer ^= 1u;

        DMA_Cmd(LIDAR_USART_RX_CH, DISABLE);
        DMA_SetCurrDataCounter(LIDAR_USART_RX_CH, LIDAR_DMA_RX_LEN);
        LIDAR_USART_RX_CH->MADDR = (uint32_t)g_lidar_dma.rx_buf[g_lidar_dma.dma_use_buffer];
        DMA_Cmd(LIDAR_USART_RX_CH, ENABLE);

        (void)RPLIDAR360_InputBytesFromISR(&g_rplidar,
                                           g_lidar_dma.rx_buf[oldbuffer],
                                           LIDAR_DMA_RX_LEN);
    }
}

/*=============================================================================
 * 对外应用程序接口（API）
 *============================================================================*/

/**
 * @brief 初始化RPLIDAR驱动（配置USART、DMA、启动扫描）
 */
void APP_RPLIDAR_Init(void)
{
    memset(&g_lidar_dma, 0, sizeof(g_lidar_dma));
    LIDAR_USART1_Init(115200u);
    LIDAR_DMA_RX_Init();
    RPLIDAR360_DriverInit(&g_rplidar,
                          g_rplidar_ring_buf,
                          LIDAR_RING_LEN,
                          RPLIDAR_PortSend,
                          RPLIDAR_PortDelayMs);
    Delay_Ms(800u);            /* 等待雷达稳定 */
    LIDAR_DMA_RX_Reset();
    (void)RPLIDAR360_StartScan(&g_rplidar);
}

/**
 * @brief 主任务（需在循环中周期调用，处理协议解析）
 */
void APP_RPLIDAR_Task(void)
{
    RPLIDAR360_Task(&g_rplidar);
}

/**
 * @brief 重新启动扫描（可用于异常恢复）
 * @return RPLIDAR_OK 或错误码
 */
int APP_RPLIDAR_Start(void)
{
    LIDAR_DMA_RX_Reset();
    return RPLIDAR360_StartScan(&g_rplidar);
}

/**
 * @brief 停止扫描
 * @return RPLIDAR_OK 或错误码
 */
int APP_RPLIDAR_Stop(void)
{
    return RPLIDAR360_Stop(&g_rplidar);
}

/**
 * @brief 复位雷达（软复位）
 * @return RPLIDAR_OK 或错误码
 */
int APP_RPLIDAR_Reset(void)
{
    return RPLIDAR360_Reset(&g_rplidar);
}

/**
 * @brief 拷贝当前帧数据（包含所有360个bin及元数据）
 * @param out_frame 输出指针
 * @return 1成功，0失败
 */
uint8_t APP_RPLIDAR_CopyCurrentFrame(RPLIDAR360_Frame_t *out_frame)
{
    if (out_frame == 0) return 0u;
    memcpy(out_frame, &g_rplidar.box.frame, sizeof(RPLIDAR360_Frame_t));
    return 1u;
}


uint8_t APP_RPLIDAR_CopyUsableFrame(RPLIDAR360_Frame_t *out_frame)
{
    RPLIDAR360_Frame_t frame;

    if (out_frame == 0) return 0u;

    memcpy(&frame, &g_rplidar.box.frame, sizeof(RPLIDAR360_Frame_t));
    if (RPLIDAR360_IsFrameUsableForDrive(&frame) == 0u)
    {
        return 0u;
    }

    memcpy(out_frame, &frame, sizeof(RPLIDAR360_Frame_t));
    return 1u;
}

/**
 * @brief 读取全部360个bin数据以及统计信息
 * @param out_bins       输出bin数组（长度360）
 * @param filled_count   返回已填充的有效角度数
 * @param raw_point_count返回本周期接收到的原始点数
 * @param round_seq      返回帧序号
 * @return 1成功，0失败
 */
uint8_t APP_RPLIDAR_ReadAllBins(RPLIDAR360_Bin_t out_bins[RPLIDAR360_BIN_COUNT],
                                uint16_t *filled_count,
                                uint16_t *raw_point_count,
                                uint32_t *round_seq)
{
    if (out_bins == 0) return 0u;
    memcpy(out_bins, g_rplidar.box.frame.bin, sizeof(g_rplidar.box.frame.bin));
    if (filled_count != 0) *filled_count = g_rplidar.box.frame.filled_count;
    if (raw_point_count != 0) *raw_point_count = g_rplidar.box.frame.raw_point_count;
    if (round_seq != 0) *round_seq = g_rplidar.box.frame.round_seq;
    return 1u;
}

/**
 * @brief 清空当前分箱数据（强制开始新的一圈）
 */
void APP_RPLIDAR_ClearBoxData(void)
{
    RPLIDAR360_BoxClearData(&g_rplidar.box);
}