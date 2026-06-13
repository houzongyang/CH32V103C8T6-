#ifndef LASER_H
#define LASER_H

#include <stdint.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef Rx3Size
#define Rx3Size 64
#endif

#define USART3_RX_CH              DMA1_Channel3
#define USART3_RX_DMA_IRQn        DMA1_Channel3_IRQn
#define USART3_RX_DMA_TC_FLAG     DMA1_IT_TC3
/* State = 2 时，认为距离无穷大 */
#define LASER_DISTANCE_INF_MM     ((uint16_t)0xFFFFu)
typedef enum
{
    LASER_STATE_WAIT_FRAME = 0,
    LASER_STATE_PARSE_FRAME,
    LASER_STATE_DISPLAY
} LASER_State_t;

typedef struct
{
    volatile uint8_t frame_ready;
    volatile uint16_t rx_len;

    uint16_t distance_mm;
    uint8_t valid;

    LASER_State_t state;
    uint32_t last_display_ms;
} LASER_Data_t;

extern uint8_t RX3_Buffer[Rx3Size];
extern LASER_Data_t Laser_Data;

void USART3_CFG(uint32_t baudrate);
void USART3_DMA1_CH3_INIT(void);

void Laser_Init(uint32_t baudrate);
void Laser_Task(uint32_t now_ms);
uint8_t Laser_GetDistance(uint16_t *distance_mm);

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif