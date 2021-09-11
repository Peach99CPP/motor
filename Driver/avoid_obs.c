#include "avoid_obs.h"
#include "time_cnt.h"

#include "delay.h"

#define THRESHOLD 2000

#define US
uint16_t raw_data, distance;

static Testime time_obj;
static uint8_t wait_fall = 0;
static GPIO_InitTypeDef US_GPIO_InitStruct = {0};


uint16_t distance_convert(uint16_t raw_data)
{
#ifdef US

    return  raw_data * (0.340) / 2; //转换成毫米单位
#endif
    return 0;

}
void avoid_init(void)
{
    HAL_GPIO_WritePin(US_RECEIVE_GPIO_Port, US_RECEIVE_Pin, GPIO_PIN_RESET);
    US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
    US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    US_GPIO_InitStruct.Pull = GPIO_PULLUP;
    US_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
}
void avoid_callback(void)
{
    if(!wait_fall)
    {
        wait_fall = 1;
        Get_Time_Period(&time_obj);

        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
    }
    else
    {
        Get_Time_Period( &time_obj);
        raw_data =  time_obj.Time_Delta;
        distance = distance_convert(raw_data);
        if(distance > THRESHOLD)
            distance = 0;
        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
        wait_fall = 0;
    }

}
void start_avoid(void)
{
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_RESET);
    while(distance == 0);//阻塞式
    return ;
}
