#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "chassis.h"
void chassis_task(void const * argument)
{
    while(1)
    {
        chassis_synthetic_control();
        osDelay(10);
    }
}
