#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "atk_imu.h"

extern osMessageQId IMU_QueueHandle;


void IMU_decode(void const * argument)
{
    while(1)
    {
        osDelay(1);
//        result=osMessageGet(IMU_QueueHandle,osWaitForever);
//        if(result.status == osEventMessage)
//        {
//            if(imu901_unpack(result.value.v))
//            {
//                if(rxPacket.startByte2 == UP_BYTE2) 
//                    atkpParsing(&rxPacket);
//                osDelay(1);
//            }
//        }
    }
}
