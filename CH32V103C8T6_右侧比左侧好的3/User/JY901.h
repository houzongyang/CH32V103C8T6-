#ifndef __JY901_H_
#define __JY901_H_

#include <stdint.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RX2_Len                 44u
#define JY901_FRAME_LEN         11u
#define JY901_FRAME_COUNT       4u

#define USART2_RX_CH            DMA1_Channel6
#define USART2_RX_DMA_IRQn      DMA1_Channel6_IRQn
#define USART2_RX_DMA_TC_FLAG   DMA1_IT_TC6

#define WINDOW_SIZE             50u

struct SAcc
{
    short a[3];
    short T;
};

struct SGyroRaw
{
    short w[3];
    short T;
};

struct SAngle
{
    short Angle[3];
    short T;
};

struct SMag
{
    short h[3];
    short T;
};

typedef struct
{
    float angle[3];     /* angle[0]=Roll, angle[1]=Pitch, angle[2]=Yaw原始角 */
} Angle;

typedef struct
{
    float a[3];
} Acc;

typedef struct
{
    float w[3];
} SGyro;

typedef struct
{
    float h[3];
} SMag;

typedef struct
{
    volatile uint8_t Rx_flag;
    volatile uint8_t Rx_len;

    uint8_t RxBuffer[RX2_Len];

    Angle angle;
    Acc acc;
    SGyro w;
    SMag h;

    /*
     * 航向角滤波结果
     */
    float yaw_raw_deg;          /* 原始 yaw，范围 -180~180 */
    float yaw_filter_deg;       /* 滤波 yaw，范围 -180~180 */
    float yaw_zero_deg;         /* 零点 */
    float yaw_relative_deg;     /* 相对零点 yaw，范围 -180~180 */
    uint8_t yaw_valid;

} JY901_DATA;

typedef struct
{
    float buffer[WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
    float sum;
} MovingAverage;

typedef enum
{
    JY901_STATE_WAIT_FRAME = 0,
    JY901_STATE_PARSE_FRAME,
    JY901_STATE_DISPLAY
} JY901_State_t;

extern JY901_DATA JY901_data;

void USART2_CFG(uint32_t baudrate);
void USART2_DMA1_CH6_INIT(void);

void JY901_Init(uint32_t baudrate);
void JY901_Task(uint32_t now_ms);
void JY901_SetDisplayEnabled(uint8_t enable);

/*
 * 兼容旧接口。
 * 如果旧代码调用 JY901_Process()，内部直接走一次解析。
 */
void JY901_Process(void);

void MA_Init(MovingAverage *ma);
float MA_Update(MovingAverage *ma, float new_value);

/*
 * 航向角相关接口
 */
void JY901_YawFilterReset(void);
void JY901_SetYawZero(void);

float JY901_GetYawRaw(void);
float JY901_GetYawFiltered(void);
float JY901_GetYawRelative(void);
float JY901_GetStableHeading(void);
uint8_t JY901_IsYawValid(void);

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif