
#include "SR04.h"

void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
u16 count=0;

u16 rising_cnt = 0;
u16 falling_cnt = 0;
u16 echo_flag = 0;
u16 startFlag=0;
void Input_Capture_SR40init()
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);  // 将映射到PA15
     //Echo PA15    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Trig PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    TIM_TimeBaseInitStructure.TIM_Period = 65535;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1;  // 1000_000 1us
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直连  CC1S[1:0]

    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; // 下降沿
    TIM_ICInitStructure.TIM_ICSelection =TIM_ICSelection_IndirectTI;  // 间接  CC1S[1:0]

    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2 global Interrupt 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3 ;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
     TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
     TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    //  TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);    
    

    
    TIM2->INTFR = 0;  // 中断状态清0
       // 开启中断
   

  

   
    TIM_Cmd(TIM2, ENABLE);
}
/*******************************************************************************
* Function Name  : TIM2_IRQHandler
*******************************************************************************/
void TIM2_IRQHandler(void)
{
    //  TIM_ClearITPendingBit(TIM2,TIM_IT_COM);
    if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET)  // 捕获上升沿
    {
        rising_cnt = TIM_GetCapture1(TIM2);
        startFlag=1;  // 标记开始
        TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);//清除中断标志
        
    }
    if(TIM_GetITStatus(TIM2,TIM_IT_CC2)!=RESET){ // 捕获下降沿
        falling_cnt= TIM_GetCapture2(TIM2);
        startFlag=0; // 标记结束
        TIM_ClearITPendingBit(TIM2,TIM_IT_CC2);//清除中断标志
    }
    if(TIM_GetITStatus(TIM2,TIM_IT_Update )!=RESET){ // 溢出中断捕获
        if (startFlag==1) {  // 标记在工作中
            count++;
        }        
       TIM_ClearITPendingBit(TIM2,TIM_IT_Update );//清除中断标志 
    }
}
float UltrasonicRFlength(void){
    
   
    float distance =0;
    TIM_SetCounter(TIM2,0);  // count 
    TIM_SetCompare1(TIM2,0); // cc1
    TIM_SetCompare2(TIM2,0); // cc2
    Start_Trig();
    

 
    if(count > 0){       
        distance=(65536*count+falling_cnt)*0.017;
        count=0; 
        falling_cnt=0;
        rising_cnt=0;
        return distance;
             
    } else{       
        distance = (falling_cnt - rising_cnt) * 0.017;
        count=0; 
        falling_cnt=0;
        rising_cnt=0;          
        return  distance; 
       
       
    } 
     
}
/*********************************************************************
 * @fn     
 *
 * @brief   Initializes TIM2 input capture.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void Input_Capture_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
     // PA3 ->TIM2_CH4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
     //Echo PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //Trig PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_3 |GPIO_Pin_4);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};
   

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2   , ENABLE); 
     TIM_DeInit(TIM2);
    TIM_TimeBaseInitStructure.TIM_Period = arr;   // 1000_000
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;   // 10_000  
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE ); //使能TIM2更新中断
    TIM_Cmd( TIM2, DISABLE );                    //定时器使能
}

void ENABLE_TIM(void)
{
    //while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==RESET)
    {
        TIM_SetCounter(TIM2,0);
        count = 0;
        TIM_Cmd(TIM2,ENABLE);//回响信号到来，开启定时器计数
    }
}

void DISABLE_TIM(void)
{
    //while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==SET)
    {
        TIM_Cmd(TIM2,DISABLE);//回响信号到来，开启定时器计数
    }
}

u32 GetCount(void)
{
    u32 t = 0;
    t = count*1000;
    t += TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2,0);
    Delay_Ms(10);
    return t;
}
void Start_Trig(void)
{
    Trig_H;
    Delay_Us(20);
    Trig_L;
}
//一次获取超声波测距数据 两次测距之间需要相隔一段时间，隔断回响信号
//为了消除余震的影响，取五次数据的平均值进行加权滤波。
float Ultrasoniclength(void )
{
    u32 t = 0 ;
    int i = 0;
    float length = 0 ;
//   float sum = 0;
//       Trig_H;
//       Delay_Us(20);
//       Trig_L;
       Start_Trig();
    // while (i!=5) { 
       while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==RESET){

       };  //此处一直等，等到为1，进行下一步
       ENABLE_TIM();//回响信号到来，开启定时器计数
        i = i + 1;
       while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==SET){

       } ; //此处一直等，等到为0，进行下一步，这两段while之间的时间就是高电平时间，即发出到返回接收的时间
       DISABLE_TIM();//回响信号到来，关闭定时器计数
       t = TIM_GetCounter(TIM2);
       length=(t+count*1000)/58.0;//通过回响信号计算距离
    //    sum = length + sum ;
    //    printf("sum is %3.2f,length is %3.2f,t is %d,count is %d\r\n",sum,length,t,count);
       TIM_SetCounter(TIM2,0);
       count = 0;
       Delay_Ms(50);
    // } 
    
     return  length; 
}





