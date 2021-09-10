#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "avoid_obs.h"
#include "delay.h"
#include "main.h"

void avoid_task(void const * argument)
{
    while(1)
    {
        HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_SET);
        delay_us(15);
        HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_RESET);
        osDelay(100);
    }
}
