#include "uart_handle.h"
#include "FreeRTOS.h"
#include "queue.h"

extern uint32_t vPortGetIPSR(void);//使用该函数来查看是否在中断中

//根据使用的调试器类型进行选择
#define USE_BLE
//#define USE_ATK

#define MAX_BUFFER_SIZE 100
#define MAX_SIZE 200
uint16_t USART_RX_STA = 0;
uint32_t rec_count = 0;
uint8_t USART_RX_BUF[MAX_SIZE];
QueueHandle_t tx_queue ;

#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数,使用队列来进行printf,此时要另外开一个task循环�?
int fputc(int ch, FILE *f)
{
    if(tx_queue != NULL)
    {
        if(vPortGetIPSR())
        {
            BaseType_t if_higher_woken = pdFALSE;
            while( xQueueSendFromISR(tx_queue, &ch, &if_higher_woken) != pdTRUE); //阻塞式。确保此时已经成功把消息放入队列
            portYIELD_FROM_ISR(if_higher_woken);//判断是否需要进行任务调�?
        }
        else
        {
            //此时并不是在中断中被调用，可以直接写入数�?
            xQueueSend(tx_queue, &ch, 1);
        }
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    return ch;
}
#endif
void printf_init(void)
{
    tx_queue = xQueueCreate(MAX_SIZE, sizeof(uint8_t));
}
/**********************************************************************
  * @Name    USART1_IRQHandler
  * @功能说明 usrat1 handler
  * @param   None
  * @返回�?
  * @author  peach99CPP
***********************************************************************/

void U1_IRQHandler(void)
{
    uint8_t rec;
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        rec =  huart1.Instance->RDR;
        if(!(USART_RX_STA & 0x8000))//接收未完�?
        {
#ifdef USE_BLE
            if(rec == 0x0a || rec == 0x21)//兼容非电脑设备的串口发�? 以！为结�?

            {
                USART_RX_STA |= 0x8000;
                return ;
            }
            if( rec == 0x0d ) return ;
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3fff] = rec;
                USART_RX_STA++;
                if(USART_RX_STA & USART_RX_STA & 0x3fff == MAX_SIZE) USART_RX_STA = 0;
            }
#endif
#ifdef USE_ATK
            if(rec == 0x0d)
            {
                USART_RX_STA |= 0x4000;
                return;
            }
            if( USART_RX_STA & 0x4000 && rec == 0x0a)
            {
                USART_RX_STA |= 0x8000;
                return;
            }
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3fff] = rec;
                USART_RX_STA++;
                if(USART_RX_STA & USART_RX_STA & 0x3fff == MAX_SIZE) USART_RX_STA = 0;
            }

#endif
        }
    }
    else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TXE);
        BaseType_t xTaskWokenByReceive = pdFALSE;
        //发送队列中有数据需要发�?
        if (xQueueReceiveFromISR(tx_queue, (void*)&rec, &xTaskWokenByReceive) == pdPASS)
            huart1.Instance->TDR = rec;
        else
            //无数据发送就关闭发送中�?
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
    else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))//处理ORE错误导致卡死在中断里
    {
        uint8_t tmp;
        tmp = USART1->ISR;
        tmp = USART1->RDR;
        (void)tmp;
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
    }
}


