#include "printf_task.h"
extern QueueHandle_t tx_queue ;
//void printfqueue(void const * argument)
//{
//    while(1)
//    {
//        uint8_t ch = 0;
//        if(xQueueReceive(tx_queue, &ch, 1) == pdTRUE)
//        {
//            while((USART1->ISR & 0X40) == 0);
//            USART1->TDR = (uint8_t)ch;
//        }
//    }
//}
