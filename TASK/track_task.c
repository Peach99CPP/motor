#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "track_bar_receive.h"
void track_scan(void const * argument)
{
    while(1)
    {
        if(dma_count >0)
        {
            track_decode();
        }
          osDelay(1);
    }
}
