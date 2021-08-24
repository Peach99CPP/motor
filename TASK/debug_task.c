#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
void Startdebug(void const * argument)
{
    while(1)
    {
        motor_debug();
        osDelay(1);
////        set_speed(0,20,0);
////        osDelay(3000);
////        set_speed(0,0,0);
////        osDelay(5000);
//        osDelay(1);

        
    }
}
