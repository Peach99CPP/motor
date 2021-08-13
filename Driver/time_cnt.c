#include "time_cnt.h"
#include "stm32f7xx_hal.h"
#include "usmart.h"
#include "chassis.h"
static TIM_HandleTypeDef htim6;
volatile uint32_t TIME_ISR_CNT;
//系统时间
Time_t Time_Sys;
long tim6_tick;
/**********************************************************************************************************
*函 数 名: Get_Time_Init
*功能说明: 时间周期计数模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Init(void)
{
    //使能定时器时钟
    __HAL_RCC_TIM6_CLK_ENABLE();

    //定时器初始化
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 108 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10000 - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);

    //使能定时器中断
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    //启动定时器
    HAL_TIM_Base_Start_IT(&htim6);
}

/**********************************************************************************************************
*函 数 名: TIM6_IRQHandler
*功能说明: 定时器6中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM6_DAC_IRQHandler(void)
{
    static uint16_t Microsecond_Cnt = 0;

    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
    {

        tim6_tick++;//10ms一个单位
        if(tim6_tick % 10 == 0)
        {
            usmart_dev.scan();
        }
        if(tim6_tick > 65535) tim6_tick = 0;
        show_speed();

        chassis_synthetic_control();


        //每10ms自加
        TIME_ISR_CNT++;
        Microsecond_Cnt += 10;
        //1秒
        if (Microsecond_Cnt >= 1000)
        {
            Microsecond_Cnt = 0;
            Time_Sys.second++;
            //1分钟
            if (Time_Sys.second >= 60)
            {
                Time_Sys.second = 0;
                Time_Sys.minute++;
                //1小时
                if (Time_Sys.minute >= 60)
                {
                    Time_Sys.minute = 0;
                    Time_Sys.hour++;
                }
            }
        }
        Time_Sys.microsecond = Microsecond_Cnt;
        __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);
    }

}
/**********************************************************************************************************
*函 数 名: Get_Period
*功能说明: 获取时间周期
*形    参: 时间周期结构体
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Period(Testime *Time_Lab)
{
    //如果还未初始化
    if (Time_Lab->inited == 0)
    {
        Time_Lab->inited = 1;
        Time_Lab->Last_Time = Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM6->CNT;
        Time_Lab->Time_Delta = 0;
    }
    Time_Lab->Last_Time = Time_Lab->Now_Time;
    //单位us
    Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM6->CNT;
    Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
}
