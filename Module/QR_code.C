#include "QR_code.h"

uint8_t head_cmd[4] = {'H', 'E', 'A', 'D'};//协议的开头
uint8_t tail_cmd[4] = {'T', 'A', 'I', 'L'};
#define QR_BUFFER_SIZE 9 //一帧数据有多少个字节
#define BUFFER_END (QR_BUFFER_SIZE-1) //数据的结尾，下标形式

extern UART_HandleTypeDef huart3;//避免出现不必要的警告

QR_t QR =
{
    .QR_uart = &huart3,
    .rec_len = 0,
    .RX_OK = 0,
    .RX_data = {0},
    .color = init_status
};//创建结构体并赋值=初值





/**********************************************************************
  * @Name    QR_receive
  * @declaration : 将该函数放在串口的IRQhandler中进行执行，用于处理数据的接收
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void QR_receive(void)
{
    static uint8_t rec;
    if(__HAL_UART_GET_FLAG(QR.QR_uart, UART_FLAG_RXNE)) //接收
    {
        rec = QR.QR_uart->Instance->RDR;
        if(!QR.RX_OK)//接收未完成
        {
            QR.RX_data[QR.rec_len ++] = rec;//直接存进去
            if(QR.rec_len >= QR_BUFFER_SIZE) //读取到的个数达到要求
            {
                QR.RX_OK = 1;//接收完成
                QR_decode();//开始解析
            }
        }

    }
}


/**********************************************************************
  * @Name    QR_decode
  * @declaration : 该函数用于将上一函数收到的数据进行解析吗，得到其中的数据
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void QR_decode(void)
{
    //判断帧头帧尾是否检验通过
    if(     QR.RX_data[0] == head_cmd[0]    && \
            QR.RX_data[1] == head_cmd[1]    && \
            QR.RX_data[2] == head_cmd[2]    && \
            QR.RX_data[3] == head_cmd[3]    && \
            QR.RX_data[BUFFER_END - 3] == tail_cmd[0] && \
            QR.RX_data[BUFFER_END - 2] == tail_cmd[1] && \
            QR.RX_data[BUFFER_END - 1] == tail_cmd[2] && \
            QR.RX_data[BUFFER_END] == tail_cmd[3] \
      )
    {
        //解析本次获得的数据
        if(QR.RX_data[4] == 'R') QR.color = red; //红色
        else if(QR.RX_data[4] == 'B') QR.color = blue; //蓝色
        else QR.color = init_status; //接收到了其他数据，此状态既是初始状态又是错误标记
    }
    QR.rec_len = 0;//清除计数器
    QR.RX_OK = 0;//清空标志位
    memset(QR.RX_data, 0, sizeof(QR.RX_data));//把缓存的内容全部清除
}
