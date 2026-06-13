#include "Motor.h"


/***

      
*/
//  
/*********************************************************************
 * @fn      TIM1__Init
 *
 * @brief   Initializes TIM1 complementary output 
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_Motor_Init()
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_OCInitTypeDef       TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
    // PA8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 10k
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1; //  1000_000  
    TIM_TimeBaseInitStructure.TIM_Period = 100-1; // 10_000   10Kh
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void Car_SetRearSpeed(uint16_t duty)
{
    if (duty > 100)
    {
        duty = 100;
    }

    TIM_SetCompare1(TIM1, duty);
}