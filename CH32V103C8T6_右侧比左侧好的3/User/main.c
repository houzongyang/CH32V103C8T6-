

#include "debug.h"
#include "Oled.h"
#include "SR04.h"
#include "Servo.h"
#include "Motor.h"

#include "rplidar_360.h"
#include "cone_detect.h"

#include "string.h"
#include "stdio.h"
#include "GPS.h"
#include "JY901.h"
#include "Laser.h"

#define LIDAR_ALL_BINS_PRINT_PERIOD_SEC      10u
#define LIDAR_ALL_BINS_PRINT_PERIOD_MS       ((uint32_t)LIDAR_ALL_BINS_PRINT_PERIOD_SEC * 1000u)
#define LASER_USART3_BAUDRATE     115200u
#define JY901_USART2_BAUDRATE     9600u


/////////////////////00000000000000000000000000000000000000000000




static ConeDriveConfig_t drive_cfg;
static ConeDriveContext_t drive_ctx;

static int32_t APP_GpsJitterMicroDeg(uint32_t *seed, int32_t range)
{
    *seed = (*seed * 1103515245u) + 12345u;
    return (int32_t)((*seed >> 16) % (uint32_t)((range * 2) + 1)) - range;
}

static void APP_FormatCoord(char *buf, uint8_t len, const char *prefix, int32_t micro_deg)
{
    int32_t deg = micro_deg / 1000000;
    int32_t frac = micro_deg % 1000000;

    if(frac < 0)
    {
        frac = -frac;
    }

    snprintf(buf, len, "%s%ld.%06ld", prefix, (long)deg, (long)frac);
}

static void APP_ShowFixedLocation(void)
{
    static uint32_t gps_seed = 0x13572468u;
    char line[20];
    int32_t lon = 116167757 + APP_GpsJitterMicroDeg(&gps_seed, 18);
    int32_t lat = 39725120 + APP_GpsJitterMicroDeg(&gps_seed, 18);

    OLED_Clear();
    OLED_ShowString(1, 1, (char *)"GPS Sim");
    APP_FormatCoord(line, sizeof(line), "Lon:", lon);
    OLED_ShowString(2, 1, line);
    APP_FormatCoord(line, sizeof(line), "Lat:", lat);
    OLED_ShowString(3, 1, line);
}


/*=============================================================================
 * SysTick 1ms 系统计时
 *
 * 说明:
 * - 参�?WCH 示例，SysTick 时钟源使�?HCLK/8�? * - 比较�?ticks = SystemCoreClock / 8 / 1000 - 1，所以中断周期为 1ms�? * - 中断内只做计数，�?print，不做耗时操作�? *============================================================================*/

volatile uint32_t g_system_ms = 0;

/* 系统 SysTick - 1ms 计时 */
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


static void SYSTICK_ClearCounter(void)
{
    SysTick->CNTL0 = 0;
    SysTick->CNTL1 = 0;
    SysTick->CNTL2 = 0;
    SysTick->CNTL3 = 0;

    SysTick->CNTH0 = 0;
    SysTick->CNTH1 = 0;
    SysTick->CNTH2 = 0;
    SysTick->CNTH3 = 0;
}

void SYSTICK_Init_Config(uint64_t ticks)
{
    SysTick->CTLR = 0x0000u;          /* 关闭系统计数�?*/

    SYSTICK_ClearCounter();

    SysTick->CMPLR0 = (u8)(ticks & 0xFFu);
    SysTick->CMPLR1 = (u8)((ticks >> 8) & 0xFFu);
    SysTick->CMPLR2 = (u8)((ticks >> 16) & 0xFFu);
    SysTick->CMPLR3 = (u8)((ticks >> 24) & 0xFFu);

    SysTick->CMPHR0 = (u8)((ticks >> 32) & 0xFFu);
    SysTick->CMPHR1 = (u8)((ticks >> 40) & 0xFFu);
    SysTick->CMPHR2 = (u8)((ticks >> 48) & 0xFFu);
    SysTick->CMPHR3 = (u8)((ticks >> 56) & 0xFFu);

    NVIC_SetPriority(SysTicK_IRQn, 15);
    NVIC_EnableIRQ(SysTicK_IRQn);

    /*
     * 使用 HCLK/8 作为 SysTick 时钟源�?     * bit0 STE  : 使能计数�?     * bit1 STIE : 使能计数中断
     * bit3 STRE : 计数到比较值后自动重装
     * bit2 STCLK 保持 0，即 HCLK/8�?     */
    SysTick->CTLR = (1u << 0) | (1u << 1) | (1u << 3);
}

static void APP_SysTick1ms_Init(void)
{
    uint64_t ticks;

    g_system_ms = 0;

    ticks = (uint64_t)SystemCoreClock / 8u / 1000u;
    if (ticks > 0u)
    {
        ticks -= 1u;
    }

    SYSTICK_Init_Config(ticks);
}

static uint32_t APP_SysTickGetMs(void)
{
    return g_system_ms;
}

void SysTick_Handler(void)
{
    /*
     * 不在中断�?print�?     * �?WCH 示例清零 64bit 计数器，避免下一次计时偏移�?     */
    SYSTICK_ClearCounter();
    g_system_ms++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////






/*=============================================================================
 * 主函�?- 系统入口
 *============================================================================*/

int main(void)
{

    MovingAverage Ult_filter;   /* 超声波数据滑动平均滤波器 */

    uint32_t now_ms ;
    uint32_t last_lidar_output_ms;
    uint32_t last_oled_switch_ms;
    uint8_t show_gps_screen;
    uint8_t yaw_zero_done;



    /*-----------------------------------------------------------------------
     * 1. 系统基础初始�?     *-----------------------------------------------------------------------*/
    
    /* 中断优先级分�? 抢占优先�?位，子优先级3�?(CH32V103) */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    /* 更新系统核心时钟频率(用于延时函数) */
    SystemCoreClockUpdate();
    
    /* 初始化延时函�?基于SysTick) */
    Delay_Init();


    /*-----------------------------------------------------------------------
     * 2. 舵机(转向)初始�?     *    硬件: J25 接口, 使用 PWM9 (TIM3)
     *-----------------------------------------------------------------------*/
    TIM3_PWMOut_Init();
    turnAngle(0);   /* 初始角度回中(正前�? */
   
    /*-----------------------------------------------------------------------
     * 3. 电机(后轮驱动)初始�?     *    硬件: J31 接口, 使用 TIM1
     *-----------------------------------------------------------------------*/
    TIM1_Motor_Init();
    Car_SetRearSpeed(0u);   /* 初始速度0 */



    /*-----------------------------------------------------------------------
     * 4. OLED显示初始�?     *    硬件: J17 接口, I2C1 (SCL1/SDA1)
     *-----------------------------------------------------------------------*/
    OLED_Init();
    APP_ShowFixedLocation();
    /*-----------------------------------------------------------------------
     * 5. 超声波传感器初始�?预留，可用于近距离避�?
     *-----------------------------------------------------------------------*/
    MA_Init(&Ult_filter);


    // WT530d
    Laser_Init(LASER_USART3_BAUDRATE);

    /*
     * 初始�?JY901�?     * USART2 + DMA 接收
     */
    JY901_Init(JY901_USART2_BAUDRATE);

    /*
     * �?JY901 输出稳定一点�?     */
    Delay_Ms(300);

       /*
     * 可选：
     * 把当前车头方向设为相�?0 度�?     * 如果你想显示绝对航向角，可以注释掉这一行�?     */
    /* JY901_SetYawZero() is delayed until yaw data is valid. */
    /*-----------------------------------------------------------------------
     * 8. RPLIDAR 雷达导航系统初始�?     *    硬件: USART1, DMA1通道5, 空闲中断
     *    功能: 点云采集、角度箱建图、快速门检�?     *-----------------------------------------------------------------------*/
    
    APP_RPLIDAR_Init();
    /* 等待100ms，确保各外设稳定 */
    Delay_Ms(100);
  /*
     * 从这里开始启�?1ms SysTick 计时�?     * 注意: 后续主循环内不要再调�?Delay_Ms/Delay_Us，避免占�?SysTick 影响计时�?     */
   
    last_lidar_output_ms = APP_SysTickGetMs();
    last_oled_switch_ms = last_lidar_output_ms;
    show_gps_screen = 1u;
    yaw_zero_done = 0u;
    ConeDetect_Init();
    ConeDrive_ConfigDefault(&drive_cfg);

    /*
     * 根据实际车调整参�?     */
    drive_cfg.speed_turn = 38;
    drive_cfg.speed_approach = 38;
    drive_cfg.speed_centering = 38;
    drive_cfg.speed_pass = 45;

    drive_cfg.pass_time_ms = 3000;

    drive_cfg.yaw_ok_deg = 2;
    drive_cfg.center_start_distance_mm = 2400;
    drive_cfg.max_steer_angle = 42;

       /*
     * 如果车头右偏时，车反而继续右打方向，
     * 把这个改�?-1�?     */
    drive_cfg.yaw_sign = 1;
    drive_cfg.target_gate_count = 0u;
    ConeDrive_Init(&drive_ctx);
    

    APP_SysTick1ms_Init();
    /*-----------------------------------------------------------------------
     * 9. 主循�?- 系统核心控制逻辑
     *-----------------------------------------------------------------------*/
    while (1)
    {
        now_ms = APP_SysTickGetMs();
        APP_RPLIDAR_Task();
                 /*
         * 获取 SysTick 毫秒时间�?         */
      
         ConeDrive_Update(&drive_ctx, &drive_cfg,now_ms);
           /*
         * 状态机任务�?         * 不要放到中断里执行�?         */
        if (((show_gps_screen == 0u) &&
             ((uint32_t)(now_ms - last_oled_switch_ms) >= 10000u)) ||
            ((show_gps_screen != 0u) &&
             ((uint32_t)(now_ms - last_oled_switch_ms) >= 2000u)))
        {
            last_oled_switch_ms = now_ms;
            show_gps_screen = (uint8_t)(show_gps_screen == 0u);
            if (show_gps_screen != 0u)
            {
                APP_ShowFixedLocation();
            }
        }

        JY901_SetDisplayEnabled((uint8_t)(show_gps_screen == 0u));
        JY901_Task(now_ms);
        if ((yaw_zero_done == 0u) &&
            (JY901_IsYawValid() != 0u) &&
            (now_ms >= 1500u))
        {
            JY901_SetYawZero();
            yaw_zero_done = 1u;
        }

        if (show_gps_screen == 0u)
        {
            Laser_Task(now_ms);
        }

        /* 定时输出当前 360 个雷达箱数据，用于验证装箱是否正确�?*/
        if ((uint32_t)(now_ms - last_lidar_output_ms) >= LIDAR_ALL_BINS_PRINT_PERIOD_MS)
        {
            last_lidar_output_ms = now_ms;
          
        }

        
  
     

   
    } /* while(1) end */
}
