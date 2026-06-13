#include "Servo.h"


/***

            //  2ms    向右 45°
            TIM_SetCompare1(TIM3,2000);  
            //     0度 正前方
            TIM_SetCompare1(TIM3,1500);  
            //  1sm      向左 45°   
            TIM_SetCompare1(TIM3,1000);
          

*/
//  配置为20ms   保持当前为0角度
void TIM3_PWMOut_Init()
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_OCInitTypeDef       TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
   // PA6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    
    TIM_TimeBaseInitStructure.TIM_Prescaler =72-1 ;  //1000_000 1us
    TIM_TimeBaseInitStructure.TIM_Period = 20000-1;   // 20ms

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;  //  1.5ms  0°
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //  TIMx Channel1 

    TIM_CtrlPWMOutputs(TIM3, ENABLE);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}
// 转动角度
 //  2ms   45°
/**
 * @brief 控制舵机转动到指定角度
 * @param angle 目标角度，范围-45到45度
    负的  左转45度
    正的  右转45度
 */
void turnAngle(int16_t angle) {
    // 确保角度在-45到45度范围内
    if(angle > 45) {
        angle = 45;
    } else if(angle < -45) {
        angle = -45;
    }
    
    // 将角度映射到PWM值（1000-2000）
    // 线性映射公式：PWM = 1500 + (angle * (500/45))
    uint16_t pwm = 1500 + (angle * 500 / 45);
    
    // 设置PWM值
    TIM_SetCompare1(TIM3, pwm);
}