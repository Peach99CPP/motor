#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "track_bar_receive.h"
#include "uart_handle.h"

void track_scan(void const *argument)
{
    static long counter = 0;
    while (1)
    {
        if (dma_count > 0)
        {
            track_decode();
        }
        counter = counter > 65536 ? 0 : counter;
        counter++;
        osDelay(1);
    }
}
