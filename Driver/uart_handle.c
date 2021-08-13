#include "uart_handle.h"
#define MAX_BUFFER_SIZE 100
#define MAX_SIZE 200
uint16_t USART_RX_STA = 0;
uint32_t rec_count = 0;
uint8_t USART_RX_BUF[MAX_SIZE];

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
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        rec =  huart1.Instance->RDR;
        if(!(USART_RX_STA & 0x8000))//接收未完成
        {

            if((USART_RX_STA & 0X4000) )
            {
                if(rec == 0x0a)
                {
                    USART_RX_STA |= 0x8000;
                    return ;
                }
                else USART_RX_STA = 0;
            }
            if(rec == 0x0d) USART_RX_STA |= 0x4000;
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3fff] = rec;
                USART_RX_STA++;
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
  * @debug  abandoned use usmart
***********************************************************************/

//void uart1_decode(void)
//{
//    // change th value of motor_pid_param
//    set_motor_pid(uart1_rxbuffer[0] + uart1_rxbuffer[1], \
//                  uart1_rxbuffer[2], \
//                  uart1_rxbuffer[3]);
//    rec_flag = 0;//clear rec flag to enable rec data handle
//    memset(uart1_rxbuffer, 0, rec_count * sizeof(rec_count)); // claer the buffer
//    printf("setting param completed\r\n");
//}
