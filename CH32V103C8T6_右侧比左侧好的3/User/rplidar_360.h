#ifndef RPLIDAR_360_H
#define RPLIDAR_360_H

#include <stdint.h>
#include "debug.h"
#ifdef __cplusplus
extern "C" {
#endif

#define RPLIDAR360_BIN_COUNT             360u

#define RPLIDAR_OK                       0
#define RPLIDAR_ERR                     -1
#define RPLIDAR_RING_OVERFLOW           -2

typedef struct
{
    uint16_t distance_mm;
    uint8_t  quality;
    uint8_t  valid;
} RPLIDAR360_Bin_t;

typedef struct
{
    RPLIDAR360_Bin_t bin[RPLIDAR360_BIN_COUNT];
    uint16_t raw_point_count;
    uint16_t filled_count;
    uint32_t round_seq;
} RPLIDAR360_Frame_t;

void APP_RPLIDAR_Init(void);
void APP_RPLIDAR_Task(void);

int APP_RPLIDAR_Start(void);
int APP_RPLIDAR_Stop(void);
int APP_RPLIDAR_Reset(void);

/* Copy all current realtime 360-bin data. */
uint8_t APP_RPLIDAR_CopyCurrentFrame(RPLIDAR360_Frame_t *out_frame);
/* Copy only when the current frame has enough front-sector data for driving. */
uint8_t APP_RPLIDAR_CopyUsableFrame(RPLIDAR360_Frame_t *out_frame);
void APP_RPLIDAR_ClearBoxData(void);
/* Read only the 360 bins, with optional frame counters. */
uint8_t APP_RPLIDAR_ReadAllBins(RPLIDAR360_Bin_t out_bins[RPLIDAR360_BIN_COUNT],
                                uint16_t *filled_count,
                                uint16_t *raw_point_count,
                                uint32_t *round_seq);

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));;
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));;

#ifdef __cplusplus
}
#endif

#endif
