#include "Laser.h"
#include "Oled.h"
#include <string.h>
#include <stdio.h>

uint8_t RX3_Buffer[Rx3Size];
static volatile uint8_t s_laser_ever_frame = 0u;
static volatile uint8_t s_laser_parse_error = 0u;

LASER_Data_t Laser_Data = {
    0,
    0,
    0,
    0,
    LASER_STATE_WAIT_FRAME,
    0
};

static uint8_t Laser_IsDigit(uint8_t c)
{
    return ((c >= '0') && (c <= '9')) ? 1u : 0u;
}

static uint8_t Laser_AsciiToDigit(uint8_t c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t)(c - '0');
    }

    return 0u;
}

/*
 * 你原来的 Laser 距离在 src[25] ~ src[28]
 */
static uint8_t Laser_ParseDistance(const uint8_t *src,
                                   uint16_t len,
                                   uint16_t *distance_mm)
{
    uint16_t i;
    uint16_t state = 0u;
    uint8_t found_state = 0u;

    uint32_t distance = 0u;
    uint8_t found_distance = 0u;
    uint8_t digit_count = 0u;

    if ((src == 0) || (distance_mm == 0)) {
        return 0u;
    }

    if (len < 3u) {
        return 0u;
    }

    /*
     * 先解析 State;
     *
     * 例如：
     * State;0 , RangeValid
     * d:  355 mm
     *
     * State;2 , SignalFail
     * d:  439 mm
     */
    for (i = 0u; i < len; i++) {
        if ((i + 6u) < len) {
            if ((src[i] == (uint8_t)'S') &&
                (src[i + 1u] == (uint8_t)'t') &&
                (src[i + 2u] == (uint8_t)'a') &&
                (src[i + 3u] == (uint8_t)'t') &&
                (src[i + 4u] == (uint8_t)'e') &&
                (src[i + 5u] == (uint8_t)';')) {

                i = (uint16_t)(i + 6u);

                while ((i < len) && (src[i] == (uint8_t)' ')) {
                    i++;
                }

                while ((i < len) && (Laser_IsDigit(src[i]) != 0u)) {
                    state = (uint16_t)((state * 10u) + Laser_AsciiToDigit(src[i]));
                    found_state = 1u;
                    i++;
                }

                break;
            }
        }
    }

    /*
     * State = 2，SignalFail
     * 认为前方距离无穷大
     */
    if ((found_state != 0u) && (state == 2u)) {
        *distance_mm = LASER_DISTANCE_INF_MM;
        return 1u;
    }

    /*
     * 如果找到了 State，但是不是 0，也不是 2，
     * 可以认为当前数据无效。
     */
    if ((found_state != 0u) && (state != 0u)) {
        return 0u;
    }

    /*
     * State = 0 时，继续解析 d:
     *
     * 例如：
     * d:  355 mm
     */
    for (i = 0u; i < (uint16_t)(len - 1u); i++) {
        if ((src[i] == (uint8_t)'d') &&
            (src[i + 1u] == (uint8_t)':')) {

            i = (uint16_t)(i + 2u);
            found_distance = 1u;
            break;
        }
    }

    if (found_distance == 0u) {
        return 0u;
    }

    while ((i < len) && (src[i] == (uint8_t)' ')) {
        i++;
    }

    while ((i < len) && (Laser_IsDigit(src[i]) != 0u)) {
        distance = (distance * 10u) + Laser_AsciiToDigit(src[i]);
        digit_count++;
        i++;

        if (distance >= LASER_DISTANCE_INF_MM) {
            return 0u;
        }
    }

    if (digit_count == 0u) {
        return 0u;
    }

    *distance_mm = (uint16_t)distance;

    return 1u;
}

static void Laser_DMA_Restart(void)
{
    DMA_Cmd(USART3_RX_CH, DISABLE);
    DMA_SetCurrDataCounter(USART3_RX_CH, Rx3Size);
    DMA_Cmd(USART3_RX_CH, ENABLE);
}

/*
 * USART3：
 * PB10 TX：你已经用于 printf
 * PB11 RX：接 Laser TX
 */
void USART3_CFG(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART3, ENABLE);
}

void USART3_DMA1_CH3_INIT(void)
{
    DMA_InitTypeDef  DMA_InitStructure  = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(USART3_RX_CH);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RX3_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Rx3Size;

    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(USART3_RX_CH, &DMA_InitStructure);

    DMA_ITConfig(USART3_RX_CH, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_RX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(USART3_RX_CH, ENABLE);

    /*
     * 只开启 USART3 RX DMA。
     * USART3 TX 仍然可以给 printf 使用。
     */
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

void Laser_Init(uint32_t baudrate)
{
    memset(RX3_Buffer, 0, sizeof(RX3_Buffer));

    Laser_Data.frame_ready = 0u;
    Laser_Data.rx_len = 0u;
    Laser_Data.distance_mm = 0u;
    Laser_Data.valid = 0u;
    Laser_Data.state = LASER_STATE_WAIT_FRAME;
    Laser_Data.last_display_ms = 0u;

    USART3_CFG(baudrate);
    USART3_DMA1_CH3_INIT();
}

uint8_t Laser_GetDistance(uint16_t *distance_mm)
{
    if (distance_mm == 0) {
        return 0u;
    }

    if (Laser_Data.valid == 0u) {
        return 0u;
    }

    *distance_mm = Laser_Data.distance_mm;
    return 1u;
}

void Laser_Task(uint32_t now_ms)
{
    uint16_t distance;
    char show_buf[20];

    if ((now_ms - Laser_Data.last_display_ms) >= 200u) {
        Laser_Data.last_display_ms = now_ms;

        memset(show_buf, 0, sizeof(show_buf));
        if (Laser_Data.valid != 0u) {
            snprintf(show_buf, sizeof(show_buf),
                     "Laser:%4umm ",
                     Laser_Data.distance_mm);
        } else if (s_laser_ever_frame == 0u) {
            snprintf(show_buf, sizeof(show_buf),
                     "Laser:NFRM  ");
        } else if (s_laser_parse_error != 0u) {
            snprintf(show_buf, sizeof(show_buf),
                     "Laser:BAD   ");
        } else {
            snprintf(show_buf, sizeof(show_buf),
                     "Laser:----mm ");
        }

        OLED_ShowString(1, 1, show_buf);
    }

    switch (Laser_Data.state) {

    case LASER_STATE_WAIT_FRAME:
        if (Laser_Data.frame_ready != 0u) {
            Laser_Data.state = LASER_STATE_PARSE_FRAME;
        }
        break;

    case LASER_STATE_PARSE_FRAME:
        distance = 0u;

        if (Laser_ParseDistance(RX3_Buffer,
                                Laser_Data.rx_len,
                                &distance) != 0u) {
            Laser_Data.distance_mm = distance;
            Laser_Data.valid = 1u;
            s_laser_parse_error = 0u;
        } else {
            Laser_Data.valid = 0u;
            s_laser_parse_error = 1u;
        }

        memset(RX3_Buffer, 0, sizeof(RX3_Buffer));
        Laser_Data.rx_len = 0u;
        Laser_Data.frame_ready = 0u;

        Laser_DMA_Restart();

        Laser_Data.state = LASER_STATE_DISPLAY;
        break;

    case LASER_STATE_DISPLAY:
        Laser_Data.state = LASER_STATE_WAIT_FRAME;
        break;

    default:
        Laser_Data.state = LASER_STATE_WAIT_FRAME;
        break;
    }
}

void USART3_IRQHandler(void)
{
    volatile uint32_t temp;

    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {

        /*
         * 清除 IDLE 标志：先读状态寄存器，再读数据寄存器。
         */
        temp = USART3->STATR;
        temp = USART3->DATAR;
        (void)temp;

        DMA_Cmd(USART3_RX_CH, DISABLE);

        Laser_Data.rx_len =
            (uint16_t)(Rx3Size - DMA_GetCurrDataCounter(USART3_RX_CH));

        if (Laser_Data.rx_len > Rx3Size) {
            Laser_Data.rx_len = 0u;
        }

        s_laser_ever_frame = 1u;
        Laser_Data.frame_ready = 1u;
    }
}

void DMA1_Channel3_IRQHandler(void)
{
    if (DMA_GetITStatus(USART3_RX_DMA_TC_FLAG) != RESET) {

        DMA_ClearITPendingBit(USART3_RX_DMA_TC_FLAG);

        DMA_Cmd(USART3_RX_CH, DISABLE);

        Laser_Data.rx_len = Rx3Size;
        s_laser_ever_frame = 1u;
        Laser_Data.frame_ready = 1u;
    }
}