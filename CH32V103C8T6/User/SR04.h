#ifndef __SR04_H_
#define __SR04_H_

#include "debug.h"

#define  Trig_H  GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define  Trig_L  GPIO_ResetBits(GPIOA,GPIO_Pin_4)
float UltrasonicRFlength(void);
void Input_Capture_Init(u16 arr, u16 psc);
void Input_Capture_SR40init();
void ENABLE_TIM(void);
void DISABLE_TIM(void);
u32 GetCount(void);
void Start_Trig(void);
float Ultrasoniclength(void );
void TIM2_IRQHandler(void);
#endif