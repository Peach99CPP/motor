#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
void Startdebug(void const * argument)
{
    Start_Read_Switch();
    while(1)
    {
//        motor_debug();
        osDelay(1);

    }
}
