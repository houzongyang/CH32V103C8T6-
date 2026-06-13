#include "JY901.h"
#include "Oled.h"
#include <string.h>
#include <stdio.h>

JY901_DATA JY901_data;

static JY901_State_t jy901_state = JY901_STATE_WAIT_FRAME;
static uint32_t jy901_last_display_ms = 0u;
static uint8_t jy901_display_enabled = 1u;

/*
 * 航向角滤波变量
 *
 * JY901 的 yaw 一般是 -180~180。
 * 如果直接平均，179 和 -179 会被平均成 0，这是错误的。
 *
 * 所以这里先做 unwrap：
 * 179 -> 181 -> 182 这种连续角度
 * 滤波后再转回 -180~180。
 */
static MovingAverage jy901_yaw_ma;
static uint8_t jy901_yaw_filter_init = 0u;
static float jy901_yaw_last_raw = 0.0f;
static float jy901_yaw_unwrap = 0.0f;


/* ====================== 基础工具函数 ====================== */

static float JY901_NormalizeAngle180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static uint8_t JY901_CheckSum(uint8_t *buf)
{
    uint8_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < 10u; i++) {
        sum += buf[i];
    }

    return (sum == buf[10]) ? 1u : 0u;
}

static int16_t JY901_GetS16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

void MA_Init(MovingAverage *ma)
{
    uint8_t i;

    if (ma == 0) {
        return;
    }

    for (i = 0u; i < WINDOW_SIZE; i++) {
        ma->buffer[i] = 0.0f;
    }

    ma->index = 0u;
    ma->count = 0u;
    ma->sum = 0.0f;
}

float MA_Update(MovingAverage *ma, float new_value)
{
    float avg;

    if (ma == 0) {
        return new_value;
    }

    if (ma->count < WINDOW_SIZE) {
        ma->buffer[ma->index] = new_value;
        ma->sum += new_value;
        ma->count++;
    } else {
        ma->sum -= ma->buffer[ma->index];
        ma->buffer[ma->index] = new_value;
        ma->sum += new_value;
    }

    ma->index++;

    if (ma->index >= WINDOW_SIZE) {
        ma->index = 0u;
    }

    avg = ma->sum / (float)ma->count;

    return avg;
}


/* ====================== 航向角滤波 ====================== */

void JY901_YawFilterReset(void)
{
    MA_Init(&jy901_yaw_ma);

    jy901_yaw_filter_init = 0u;
    jy901_yaw_last_raw = 0.0f;
    jy901_yaw_unwrap = 0.0f;

    JY901_data.yaw_raw_deg = 0.0f;
    JY901_data.yaw_filter_deg = 0.0f;
    JY901_data.yaw_zero_deg = 0.0f;
    JY901_data.yaw_relative_deg = 0.0f;
    JY901_data.yaw_valid = 0u;
}

static void JY901_UpdateYawFilter(float yaw_raw)
{
    float delta;
    float filtered_unwrap;

    yaw_raw = JY901_NormalizeAngle180(yaw_raw);

    JY901_data.yaw_raw_deg = yaw_raw;

    if (jy901_yaw_filter_init == 0u) {
        jy901_yaw_filter_init = 1u;

        jy901_yaw_last_raw = yaw_raw;
        jy901_yaw_unwrap = yaw_raw;

        MA_Init(&jy901_yaw_ma);
        filtered_unwrap = MA_Update(&jy901_yaw_ma, jy901_yaw_unwrap);
    } else {
        /*
         * 计算当前 raw yaw 相对上一帧的最短角度差。
         * 例如：
         * last = 179
         * raw  = -179
         * delta = +2，而不是 -358
         */
        delta = JY901_NormalizeAngle180(yaw_raw - jy901_yaw_last_raw);

        jy901_yaw_unwrap += delta;
        jy901_yaw_last_raw = yaw_raw;

        filtered_unwrap = MA_Update(&jy901_yaw_ma, jy901_yaw_unwrap);
    }

    JY901_data.yaw_filter_deg = JY901_NormalizeAngle180(filtered_unwrap);

    JY901_data.yaw_relative_deg =
        JY901_NormalizeAngle180(JY901_data.yaw_filter_deg -
                                JY901_data.yaw_zero_deg);

    JY901_data.yaw_valid = 1u;
}

void JY901_SetYawZero(void)
{
    /*
     * 把当前滤波航向角作为 0 度。
     */
    JY901_data.yaw_zero_deg = JY901_data.yaw_filter_deg;
    JY901_data.yaw_relative_deg = 0.0f;
}

float JY901_GetYawRaw(void)
{
    return JY901_data.yaw_raw_deg;
}

float JY901_GetYawFiltered(void)
{
    return JY901_data.yaw_filter_deg;
}

float JY901_GetYawRelative(void)
{
    return JY901_data.yaw_relative_deg;
}

float JY901_GetStableHeading(void)
{
    float heading = JY901_data.yaw_relative_deg;
    int16_t half_deg;

    if ((heading > -2.0f) && (heading < 2.0f)) {
        return 0.0f;
    }

    if (heading >= 0.0f) {
        half_deg = (int16_t)((heading * 2.0f) + 0.5f);
    } else {
        half_deg = (int16_t)((heading * 2.0f) - 0.5f);
    }

    return (float)half_deg / 2.0f;
}

uint8_t JY901_IsYawValid(void)
{
    return JY901_data.yaw_valid;
}

void JY901_SetDisplayEnabled(uint8_t enable)
{
    jy901_display_enabled = (enable != 0u) ? 1u : 0u;
}


/* ====================== USART2 + DMA 接收 ====================== */

static void JY901_DMA_Restart(void)
{
    DMA_Cmd(USART2_RX_CH, DISABLE);
    DMA_SetCurrDataCounter(USART2_RX_CH, RX2_Len);
    DMA_Cmd(USART2_RX_CH, ENABLE);
}

void USART2_CFG(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /*
     * USART2_TX -> PA2
     * USART2_RX -> PA3
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*
     * 使用 IDLE 中断判断一批数据接收完成。
     */
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART2, ENABLE);
}

void USART2_DMA1_CH6_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(USART2_RX_CH);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)JY901_data.RxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = RX2_Len;

    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(USART2_RX_CH, &DMA_InitStructure);

    DMA_ITConfig(USART2_RX_CH, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_RX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(USART2_RX_CH, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

void JY901_Init(uint32_t baudrate)
{
    memset(&JY901_data, 0, sizeof(JY901_data));

    jy901_state = JY901_STATE_WAIT_FRAME;
    jy901_last_display_ms = 0u;

    JY901_YawFilterReset();

    USART2_CFG(baudrate);
    USART2_DMA1_CH6_INIT();
}


/* ====================== 数据解析状态机 ====================== */

static void JY901_ParseOneFrame(uint8_t *frame)
{
    uint8_t type;

    int16_t raw0;
    int16_t raw1;
    int16_t raw2;

    float yaw_raw;

    if (frame == 0) {
        return;
    }

    if (frame[0] != 0x55) {
        return;
    }

    if (JY901_CheckSum(frame) == 0u) {
        return;
    }

    type = frame[1];

    raw0 = JY901_GetS16(frame[2], frame[3]);
    raw1 = JY901_GetS16(frame[4], frame[5]);
    raw2 = JY901_GetS16(frame[6], frame[7]);

    switch (type) {

    case 0x51:
        /*
         * 加速度，单位 g。
         */
        JY901_data.acc.a[0] = (float)raw0 / 32768.0f * 16.0f;
        JY901_data.acc.a[1] = (float)raw1 / 32768.0f * 16.0f;
        JY901_data.acc.a[2] = (float)raw2 / 32768.0f * 16.0f;
        break;

    case 0x52:
        /*
         * 角速度，单位 °/s。
         */
        JY901_data.w.w[0] = (float)raw0 / 32768.0f * 2000.0f;
        JY901_data.w.w[1] = (float)raw1 / 32768.0f * 2000.0f;
        JY901_data.w.w[2] = (float)raw2 / 32768.0f * 2000.0f;
        break;

    case 0x53:
        /*
         * 角度：
         * angle[0] = Roll
         * angle[1] = Pitch
         * angle[2] = Yaw 原始航向角
         */
        JY901_data.angle.angle[0] = (float)raw0 / 32768.0f * 180.0f;
        JY901_data.angle.angle[1] = (float)raw1 / 32768.0f * 180.0f;
        JY901_data.angle.angle[2] = (float)raw2 / 32768.0f * 180.0f;

        yaw_raw = JY901_data.angle.angle[2];

        /*
         * 更新滤波航向角。
         */
        JY901_UpdateYawFilter(yaw_raw);
        break;

    case 0x54:
        /*
         * 磁场。
         */
        JY901_data.h.h[0] = (float)raw0;
        JY901_data.h.h[1] = (float)raw1;
        JY901_data.h.h[2] = (float)raw2;
        break;

    default:
        break;
    }
}

static void JY901_ParseBuffer(void)
{
    uint8_t i;
    uint8_t *frame;

    /*
     * JY901 一般连续输出：
     * 0x51, 0x52, 0x53, 0x54
     *
     * 共 4 帧，每帧 11 字节。
     */
    for (i = 0u; i < JY901_FRAME_COUNT; i++) {
        frame = &JY901_data.RxBuffer[i * JY901_FRAME_LEN];

        if (frame[0] != 0x55) {
            continue;
        }

        JY901_ParseOneFrame(frame);
    }
}

static void JY901_Display(void)
{
    char tempbuffer[24];

    /*
     * 第2行显示 Roll / Pitch。
     */
    memset(tempbuffer, 0, sizeof(tempbuffer));
    snprintf(tempbuffer,
             sizeof(tempbuffer),
             "R:%4.0f P:%4.0f ",
             JY901_data.angle.angle[0],
             JY901_data.angle.angle[1]);
    OLED_ShowString(2, 2, tempbuffer);

    /*
     * 第3行显示滤波后的航向角。
     */
    memset(tempbuffer, 0, sizeof(tempbuffer));

    if (JY901_data.yaw_valid != 0u) {
        snprintf(tempbuffer,
                 sizeof(tempbuffer),
                 "Yaw:%7.1f ",
                 JY901_data.yaw_filter_deg);
    } else {
        snprintf(tempbuffer,
                 sizeof(tempbuffer),
                 "Yaw: ------ ");
    }

    OLED_ShowString(3, 1, tempbuffer);

    /*
     * 第4行显示相对航向角。
     * 如果你不需要相对角，也可以改成显示磁场。
     */
    memset(tempbuffer, 0, sizeof(tempbuffer));

    if (JY901_data.yaw_valid != 0u) {
        snprintf(tempbuffer,
                 sizeof(tempbuffer),
                 "Head:%6.1f ",
                 JY901_GetStableHeading());
    } else {
        snprintf(tempbuffer,
                 sizeof(tempbuffer),
                 "Head: ---- ");
    }

    OLED_ShowString(4, 1, tempbuffer);
}

void JY901_Task(uint32_t now_ms)
{
    switch (jy901_state) {

    case JY901_STATE_WAIT_FRAME:
        if (JY901_data.Rx_flag != 0u) {
            jy901_state = JY901_STATE_PARSE_FRAME;
        }
        break;

    case JY901_STATE_PARSE_FRAME:
        if (JY901_data.Rx_len >= RX2_Len) {
            JY901_ParseBuffer();
        }

        JY901_data.Rx_flag = 0u;
        JY901_data.Rx_len = 0u;

        memset(JY901_data.RxBuffer, 0, sizeof(JY901_data.RxBuffer));

        JY901_DMA_Restart();

        jy901_state = JY901_STATE_DISPLAY;
        break;

    case JY901_STATE_DISPLAY:
        /*
         * OLED 显示不要太频繁。
         */
        if ((jy901_display_enabled != 0u) &&
            ((now_ms - jy901_last_display_ms) >= 200u)) {
            jy901_last_display_ms = now_ms;
            JY901_Display();
        }

        jy901_state = JY901_STATE_WAIT_FRAME;
        break;

    default:
        jy901_state = JY901_STATE_WAIT_FRAME;
        break;
    }
}

/*
 * 兼容旧接口。
 */
void JY901_Process(void)
{
    if (JY901_data.Rx_len >= RX2_Len) {
        JY901_ParseBuffer();

        JY901_data.Rx_flag = 0u;
        JY901_data.Rx_len = 0u;

        memset(JY901_data.RxBuffer, 0, sizeof(JY901_data.RxBuffer));

        JY901_DMA_Restart();
    }
}


/* ====================== 中断函数 ====================== */

void USART2_IRQHandler(void)
{
    volatile uint32_t temp;

    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {

        /*
         * 清 IDLE 标志：
         * 先读 STATR，再读 DATAR。
         */
        temp = USART2->STATR;
        temp = USART2->DATAR;
        (void)temp;

        DMA_Cmd(USART2_RX_CH, DISABLE);

        JY901_data.Rx_len =
            (uint8_t)(RX2_Len - DMA_GetCurrDataCounter(USART2_RX_CH));

        if (JY901_data.Rx_len > RX2_Len) {
            JY901_data.Rx_len = 0u;
        }

        JY901_data.Rx_flag = 1u;
    }
}

void DMA1_Channel6_IRQHandler(void)
{
    if (DMA_GetITStatus(USART2_RX_DMA_TC_FLAG) != RESET) {

        DMA_ClearITPendingBit(USART2_RX_DMA_TC_FLAG);

        DMA_Cmd(USART2_RX_CH, DISABLE);

        JY901_data.Rx_len = RX2_Len;
        JY901_data.Rx_flag = 1u;
    }
}