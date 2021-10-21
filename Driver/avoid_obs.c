#include "avoid_obs.h"
#include "time_cnt.h"

#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "chassis.h"
#include "chassis_control.h"

#define THRESHOLD 2000

#define US
uint16_t raw_data, distance;

static Testime time_obj;
static uint8_t wait_fall = 0;
static GPIO_InitTypeDef US_GPIO_InitStruct = {0};

uint16_t distance_convert(uint16_t raw_data)
{
#ifdef US
    //根据使用的型号进行修改
    return raw_data * (0.340) / 2; //转换成毫米单位
#else
    //可以实现对其他型号的兼容
#ifdef SR
    return 0;
#endif
#endif
}

/**********************************************************************
 * @Name    avoid_init
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void avoid_init(void)
{
    //配置好引脚的信息
    HAL_GPIO_WritePin(US_RECEIVE_GPIO_Port, US_RECEIVE_Pin, GPIO_PIN_RESET);
    US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
    US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    US_GPIO_InitStruct.Pull = GPIO_PULLUP;
    US_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
}

/**********************************************************************
 * @Name    avoid_callback
 * @declaration : 测距的中断函数
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void avoid_callback(void)
{
    if (!wait_fall) //首先等到的是上升沿
    {
        wait_fall = 1;              //下一步捕获下降沿
        Get_Time_Period(&time_obj); //获取当前时间
        /* 配置引脚对下降沿触发*/
        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
    }
    else
    {
        //捕获到下降沿
        Get_Time_Period(&time_obj);            //更新时间
        raw_data = time_obj.Time_Delta;        //获取时间差
        distance = distance_convert(raw_data); //由原始数据转换得到距离数据
        if (distance > THRESHOLD)              //太大，超出
            distance = 0;
        /*将引脚配置回上升沿捕获*/
        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
        wait_fall = 0;
    }
}

/**********************************************************************
 * @Name    start_avoid
 * @declaration : 进行单次的测距
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void start_avoid(void)
{
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_RESET);
    while (distance == 0)
        osDelay(10); //阻塞式
    return;
}

#define DISTANCE_THRESHOLD 0XDB
#define AVOID_OBS_LOW_SPEED 120
#include "chassis_control.h"

//方向：1往左 2往右
void Wait_For_Avoid(int dir)
{
    Comfirm_Online(dir);
    //蓝色半场的避障 经过初步测试
    set_speed(0, AVOID_OBS_LOW_SPEED, 0);
    while (distance > 0XDB)
        osDelay(5);
    set_speed(0, 0, 0);
    osDelay(100);
    move_by_encoder(1, -15);
    Wait_OKInf(2, 5000);
    move_by_encoder(2, 70);
    Wait_OKInf(2, 5000);
    move_by_encoder(1, 15);
    Wait_OKInf(2, 5000);
    if(dir ==1 )
    {
        Comfirm_Online(2);
    }
    else if(dir ==2)
    {
        Comfirm_Online(1);
    }
}
