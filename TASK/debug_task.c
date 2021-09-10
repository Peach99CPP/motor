#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"

void Startdebug(void const * argument)
{
    while(1)
    {
//        motor_debug();
        osDelay(1);

    }
}
