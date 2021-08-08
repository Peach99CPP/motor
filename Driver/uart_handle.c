#include "uart_handle.h"
#define MAX_BUFFER_SIZE 100
static uint8_t uart1_state = 0, rec_flag = 0;
uint8_t uart1_rxbuffer[MAX_BUFFER_SIZE], rec_count = 0;

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
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART1->ISR & 0X40) == 0); //循环发送,直到发送完毕
    USART1->TDR = (uint8_t) ch;
    return ch;
}
#endif

/**********************************************************************
  * @Name    USART1_IRQHandler
  * @功能说明 usrat1 handler
  * @param   None
  * @返回值
  * @author  peach99CPP
***********************************************************************/

void USART1_IRQHandler(void)
{
    uint8_t rec;
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
        rec =  huart1.Instance->RDR;
        
        if(!rec_flag)
        {
            if(uart1_state == 0)
            {
                if(rec == 0xff)
                {
                    uart1_state = 1;
                    rec_count = 0;
                }
            }
            else if(uart1_state == 1 )
            {
                if(rec == 0xff)
                {
                    uart1_state = 0;
                    rec_flag = 1;
                    uart1_decode();
                    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) printf("not cleared\r\n");
                    return ;
                }
                else
                {
                    uart1_rxbuffer[rec_count++] = rec;
                }
                if(rec_count == MAX_BUFFER_SIZE )
                {
                    rec_flag = 1;
                    uart1_state = 0;
                    uart1_decode();

                }
            }
        }
    }

}


/**********************************************************************
  * @Name    uart1_decode
  * @功能说明 对串口 1收到的数据进行解码
  * @param   None
  * @返回值  void
  * @author  peach99CPP
***********************************************************************/

void uart1_decode(void)
{
    // change th value of motor_pid_param
    set_motor_pid(uart1_rxbuffer[0] + uart1_rxbuffer[1], \
                  uart1_rxbuffer[2], \
                  uart1_rxbuffer[3]);
    rec_flag = 0;//clear rec flag to enable rec data handle
    memset(uart1_rxbuffer, 0, rec_count * sizeof(rec_count)); // claer the buffer
    printf("setting param completed\r\n");
}
