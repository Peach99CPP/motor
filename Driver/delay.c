#include "delay.h"
time_t sys_time;
/**********************************************************************
  * @Name    sys_time_init
  * @declaration : init for the time_t structure
  * @param   ptr: [输入/出]  pointer of the count var,must 1us
**			 mode: [输入/出]  count_mode  up/dowm
**			 period: [输入/出] count_period_val
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void delay_init(TIM_HandleTypeDef * cnt_tim, count_mode mode)
{
    sys_time.cnt_tim  = cnt_tim;
    sys_time.mode = mode;
    sys_time.period_val = sys_time.cnt_tim ->Init.Period;
    HAL_TIM_Base_Start(cnt_tim);
}

/**********************************************************************
  * @Name    delay_us
  * @declaration : delay function In milliseconds
  * @param   us: [输入/出]  us value
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void delay_us(uint32_t us)
{
    uint32_t old_time, now_time;
    long time_cnt;
    time_cnt = us;
    old_time = sys_time.cnt_tim->Instance->CNT;
    if( sys_time.mode == up_count)
    {
        while(1)
        {
            now_time = sys_time.cnt_tim->Instance->CNT;
            if(now_time != old_time)
            {
                if(now_time > old_time)
                {
                    time_cnt -= (now_time - old_time);
                }
                else
                {
                    time_cnt -= (sys_time.period_val + now_time - old_time);
                }
                old_time = now_time;
                if(time_cnt <= 0)
                    break;
            }
        }
    }
    else
    {
        while(1)
        {
            now_time = sys_time.cnt_tim->Instance->CNT;
            if(now_time != old_time)
            {
                if(now_time < old_time)
                {
                    time_cnt -= (old_time - now_time);
                }
                else
                {
                    time_cnt -= (old_time - now_time + sys_time.period_val);
                }
                old_time = now_time;
                if(time_cnt <= 0)
                    break;
            }

        }
    }
}

/**********************************************************************
  * @Name    delay_ms
  * @declaration : delay ms implementation based on delay_us
  * @param   ms: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 1; i <= ms; ++i)
    {
        delay_us(1000);
    }
}
