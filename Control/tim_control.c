#include "tim_control.h"
double encoder_val[5];//默认为0
short status_flag[5];//
double encoder_sum, temp_sum;
int rising_val[5], falling_val[5], direct_[5], update_count[5];
double   cap_temp_val[5];
short cap_cnt[5];
int first_flag[5];
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

#define  FORWARD    1
#define  BACKWARD   -1
#define SPEED_PARAM    100000.0
#define TIM_COUNT_VAL  10000
#define FILTER 10.0
#define THRESHOLD_ 20.0


/*******************
*@name:HAL_TIM_PeriodElapsedCallback
*@function:利用定时器来刷新任务,计算时长，只有不修改IRQHandler才能触发此函数
*@param:定时器结构体
*@return:无
**********************/
void MY_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t i = 0;
    if(htim->Instance == TIM3)
    {
        update_count[1]++;//
        update_count[2]++;//用于计算脉宽，处理捕获中途发生定时器更新事件的情况
        if(++update_count[1] >=3)
        {
            update_count[1] = 0;
            encoder_val[1] = 0;
        }
        if(++update_count[2] >=3)
        {
            update_count[2] = 0;
            encoder_val[2] = 0;
        }
    }
    else if(htim->Instance == TIM5)
    {
        if(++update_count[3] >=3)
        {
            update_count[3] = 0;
            encoder_val[3] = 0;
        }
        if(++update_count[4] >=3)
        {
            update_count[4] = 0;
            encoder_val[4] = 0;
        }
    }

}
/**********************************************************************
  * @Name    HAL_TIM_IC_CaptureCallback
  * @declaration : handle tim ic event for encoder
  * @param   htim: [输入/出] tim structure ptr
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    double temp_val = 0;
    uint32_t  flag = 0;
    if(htim == motor1.IC.Tim && htim->Channel == motor1.IC.Active_Channel)
    {
        if(!status_flag[1])//第一次捕获到上升沿
        {
            status_flag[1] = 1;//状态标志位置1，下次进中断是在下一步
            rising_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//读取此时上升沿的值
            update_count[1] = 0;//更新事件计数器 置0
            //判断方向
            if(HAL_GPIO_ReadPin(motor1.Encoder_IO.Port, motor1.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[1] = FORWARD;
            }
            else
            {
                direct_[1] = BACKWARD;
            }
//            __HAL_TIM_ENABLE_IT(motor1.IC.Tim, TIM_IT_UPDATE);
            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_FALLING);//下一次是捕获下降沿
        }
        else//捕获到下降沿
        {
            status_flag[1] = 0 ;//状态位清除
            falling_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//读取下降沿的值
            cap_temp_val[1] += (SPEED_PARAM / (falling_val[1] - rising_val[1] + TIM_COUNT_VAL * update_count[1])) * direct_[1];//计算本次得到的脉宽。反映出转速的快慢，并累加
            cap_cnt[1]++;//采样次数累加
            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING);//准备对上升沿进行采样
            if(cap_cnt[1] == FILTER)//采样次数到达了
            {
                if(!first_flag[1])//第一次，设置变量
                {
                    first_flag[1] = 1;
                    encoder_val[1] = cap_temp_val[1] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[1] / FILTER;//获取本采样周期内的平均值
                    if(!(fabs(temp_val + encoder_val[1]) < THRESHOLD_)) //没有因为毛刺发生方向跳变，有的话直接舍弃本次获得的值
                    {
                        encoder_val[1] += temp_val;
                        encoder_val[1] /= 2.0;//均值滤波
                    }
                }
//                __HAL_TIM_DISABLE_IT(motor1.IC.Tim, TIM_IT_UPDATE);
                //相关变量清0 ！记得清0！
                temp_val = 0;
                cap_cnt[1] = 0;
                cap_temp_val[1] = 0 ;
            }

        }


    }
    else if(htim == motor2.IC.Tim && htim->Channel == motor2.IC.Active_Channel )
    {

        if(!status_flag[2])
        {
            status_flag[2] = 1;
            rising_val[2] = HAL_TIM_ReadCapturedValue(motor2.IC.Tim, motor2.IC.Channel);
            update_count[2] = 0;

            if(HAL_GPIO_ReadPin(motor2.Encoder_IO.Port, motor2.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[2] = FORWARD;
            }
            else
            {
                direct_[2] = BACKWARD;
            }
//            __HAL_TIM_ENABLE_IT(motor2.IC.Tim, TIM_IT_UPDATE);
            __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[2] = 0 ;
            falling_val[2] = HAL_TIM_ReadCapturedValue(motor2.IC.Tim, motor2.IC.Channel);
            cap_temp_val[2] += (SPEED_PARAM / (falling_val[2] - rising_val[2] + TIM_COUNT_VAL * update_count[2])) * direct_[2];
            cap_cnt[2]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_RISING);
            if(cap_cnt[2] == FILTER)
            {
                if(!first_flag[2])
                {
                    first_flag[2] = 1;
                    encoder_val[2] = cap_temp_val[2] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[2] / FILTER;
                    if(!(fabs(temp_val + encoder_val[2]) < THRESHOLD_)) //没有因为毛刺发生方向跳变
                    {
                        encoder_val[2] += temp_val;
                        encoder_val[2] /= 2.0;
                    }
                }
//                __HAL_TIM_DISABLE_IT(motor2.IC.Tim, TIM_IT_UPDATE);
                temp_val = 0;
                cap_cnt[2] = 0;
                cap_temp_val[2] = 0 ;
            }

        }

    }
    else if(htim == motor3.IC.Tim &&  htim->Channel == motor3.IC.Active_Channel)
    {

        if(!status_flag[3])
        {
            status_flag[3] = 1;
            rising_val[3] = HAL_TIM_ReadCapturedValue(motor3.IC.Tim, motor3.IC.Channel);
            update_count[3] = 0;
            if(HAL_GPIO_ReadPin(motor3.Encoder_IO.Port, motor3.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[3] = FORWARD;
            }
            else
            {
                direct_[3] = BACKWARD;
            }
//            __HAL_TIM_ENABLE_IT(motor3.IC.Tim, TIM_IT_UPDATE);
            __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[3] = 0 ;
            falling_val[3] = HAL_TIM_ReadCapturedValue(motor3.IC.Tim, motor3.IC.Channel);
            cap_temp_val[3] += (SPEED_PARAM / (falling_val[3] - rising_val[3] + TIM_COUNT_VAL * update_count[3])) * direct_[3];
            cap_cnt[3]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_RISING);
            if(cap_cnt[3] == FILTER)
            {
                if(!first_flag[3])
                {
                    first_flag[3] = 1;
                    encoder_val[3] = cap_temp_val[3] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[3] / FILTER;
                    if(!(fabs(temp_val + encoder_val[3]) < THRESHOLD_)) //没有因为毛刺发生方向跳变
                    {
                        encoder_val[3] += temp_val;
                        encoder_val[3] /= 2.0;
                    }
                }
//                __HAL_TIM_DISABLE_IT(motor3.IC.Tim, TIM_IT_UPDATE);
                temp_val = 0;
                cap_cnt[3] = 0;
                cap_temp_val[3] = 0 ;
            }

        }

    }
    else if(htim == motor4.IC.Tim &&  htim->Channel == motor4.IC.Active_Channel)
    {

        if(!status_flag[4])
        {
            status_flag[4] = 1;
            rising_val[4] = HAL_TIM_ReadCapturedValue(motor4.IC.Tim, motor4.IC.Channel);
            update_count[4] = 0;
            if(HAL_GPIO_ReadPin(motor4.Encoder_IO.Port, motor4.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[4] =  FORWARD  ;
            }
            else
            {
                direct_[4] = BACKWARD;
            }
//            __HAL_TIM_ENABLE_IT(motor4.IC.Tim, TIM_IT_UPDATE);
            __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[4] = 0 ;
            falling_val[4] = HAL_TIM_ReadCapturedValue(motor4.IC.Tim, motor4.IC.Channel);
            cap_temp_val[4] += (SPEED_PARAM / (falling_val[4] - rising_val[4] + TIM_COUNT_VAL * update_count[4])) * direct_[4];
            cap_cnt[4]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_RISING);
            if(cap_cnt[4] == FILTER)
            {
                if(!first_flag[4])
                {
                    first_flag[4] = 1;
                    encoder_val[4] = cap_temp_val[4] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[4] / FILTER;
                    if(!(fabs(temp_val + encoder_val[4]) < THRESHOLD_)) //没有因为毛刺发生方向跳变
                    {
                        encoder_val[4] += temp_val;
                        encoder_val[4] /= 2.0;
                    }
                }
//                __HAL_TIM_DISABLE_IT(motor4.IC.Tim, TIM_IT_UPDATE);
                temp_val = 0;
                cap_cnt[4] = 0;
                cap_temp_val[4] = 0 ;
            }

        }


    }
}




