#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "avoid_obs.h"
#include "delay.h"
#include "main.h"

osThreadId avoid_taskHandle;
void avoid_task(void const * argument);

int avoid_task_exit = 1;//退出标�?



/**********************************************************************
  * @Name    avoid_keep
  * @declaration : 开启避障的任务
  * @param   None
  * @retval   : �?
  * @author  peach99CPP
***********************************************************************/
void avoid_keep(void)
{
    if(avoid_task_exit)//避免重复创建
    {
        avoid_task_exit = 0;
        osThreadDef(avoidTask, avoid_task, osPriorityHigh, 0, 128);
        avoid_taskHandle = osThreadCreate(osThread(avoidTask), NULL);
    }
}





/**********************************************************************
  * @Name    avoid_task
  * @declaration : 任务的实现函�?
  * @param   argument: [输入/出]
  * @retval   : �?
  * @author  peach99CPP
***********************************************************************/
void avoid_task(void const * argument)
{
    while(!avoid_task_exit)//当函数运行时
    {
        //不断拉高再拉低信号脚。来触发超声波测�?
        HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_SET);
        delay_us(15);
        HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_RESET);
        osDelay(50);
    }
    vTaskDelete(NULL);//从任务列表中删除该任�?
    avoid_taskHandle = NULL;//句柄置空
}



/**********************************************************************
  * @Name    exit_avoid
  * @declaration : 结束任务，设置标志位
  * @param   None
  * @retval   : �?
  * @author  peach99CPP
***********************************************************************/
void exit_avoid(void)
{
    avoid_task_exit = 1;
}
