#ifndef      __MOTOR_H_
#define      __MOTOR_H_

#include "debug.h"

void TIM1_Motor_Init();

/*
 * 后轮速度控制。
 * duty 范围：0~100
 */
void Car_SetRearSpeed(uint16_t duty);
#endif