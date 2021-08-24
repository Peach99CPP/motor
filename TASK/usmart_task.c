#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usmart.h"


void usmartscan(void const * argument)
{
    while(1)
    {
        usmart_dev.scan();
        osDelay(100);
    }
}
