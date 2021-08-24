#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "ahrs.h"
#include "imu.h"
void ahrs_imu_task_(void const * argument)
{
     portTickType xLastWakeTime;
	
	imu_init();
	ahrs_init();
    
    //获取系统当前节拍
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //获取imu数据
		get_imu_data();
        //姿态解算
		ahrs_update();
        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    vTaskDelete(NULL);//防止发疯
}