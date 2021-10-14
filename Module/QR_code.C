#include "QR_code.h"
#include "servo.h"
#include "openmv.h"
#include "read_status.h "

uint8_t head_cmd[4] = {'H', 'E', 'A', 'D'}; //协议的开头
uint8_t tail_cmd[4] = {'T', 'A', 'I', 'L'};
#define QR_BUFFER_SIZE 9                //一帧数据有多少个字节
#define BUFFER_END (QR_BUFFER_SIZE - 1) //数据的结尾，下标形式

int Ennable_QR = 0;
QRcolor_t Target_Color = init_status;
extern UART_HandleTypeDef huart3; //避免出现不必要的警告

QR_t QR =
    {
        .QR_uart = &huart3,
        .rec_len = 0,
        .RX_OK = 0,
        .RX_data = {0},
        .color = init_status}; //创建结构体并赋值=初值

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
    rec = QR.QR_uart->Instance->RDR;
    if (!QR.RX_OK) //接收未完成
    {
        QR.RX_data[QR.rec_len++] = rec;   //直接存进去
        if (QR.rec_len >= QR_BUFFER_SIZE) //读取到的个数达到要求
        {
            QR.RX_OK = 1; //接收完成
            QR_decode();  //开始解析
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
#define INF_INDEX 4
    //判断帧头帧尾是否检验通过
    if (QR.RX_data[0] == head_cmd[0] &&
        QR.RX_data[1] == head_cmd[1] &&
        QR.RX_data[2] == head_cmd[2] &&
        QR.RX_data[3] == head_cmd[3] &&
        QR.RX_data[BUFFER_END - 3] == tail_cmd[0] &&
        QR.RX_data[BUFFER_END - 2] == tail_cmd[1] &&
        QR.RX_data[BUFFER_END - 1] == tail_cmd[2] &&
        QR.RX_data[BUFFER_END] == tail_cmd[3])
    {
        //解析本次获得的数据
        if (QR.RX_data[INF_INDEX] == 'R' || QR.RX_data[INF_INDEX] == 'r')
        {
            QR.color = red; //红色
        }
        else if (QR.RX_data[INF_INDEX] == 'B' || QR.RX_data[INF_INDEX] == 'b')
        {
            QR.color = blue; //蓝色
        }
        else
            QR.color = init_status; //接收到了其他数据，此状态既是初始状态又是错误标记
    }
    if ((QR.color == Target_Color) && Ennable_QR) //仅在设置了对其有响应才会执行 颜色要对 并且开启响应
    {

        if (Get_Servo_Flag())
        {
            Disable_ServoFlag();  //标记此时舵控正在运行过程中，本函数在传输舵控指令中也会被调用，此处只是为了增强记忆
            Enable_StopSignal();  //使能停车信号，让动作那边执行停车操作
            switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
            {
            case LowestHeight:
                Action_Gruop(17, 1);
                break;
            case MediumHeight:
                Action_Gruop(19, 1);
                break;
            case HighestHeight:
                Action_Gruop(18, 1);
            default:
                if (Get_IFUP() == false)
                {
                    Action_Gruop(11, 1); //机械臂升起
                    Set_IFUP(true);
                }
            }
        }

        QR.color = init_status;
    }
    QR.rec_len = 0;                            //清除计数器
    QR.RX_OK = 0;                              //清空标志位
    memset(QR.RX_data, 0, sizeof(QR.RX_data)); //把缓存的内容全部清除
}

/**********************************************************************
  * @Name    Get_QRColor
  * @declaration :
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
int Get_QRColor(void)
{
    if (QR.color != init_status)
    {
        if (QR.color == red)
        {
            return red;
        }
        else if (QR.color == blue)
        {
            return blue;
        }
    }
    return init_status;
}

/**********************************************************************
  * @Name    Set_QR_Status
  * @declaration : 设置是否对二维码内容响应
  * @param   status: [输入/出] 是否开启
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Set_QR_Status(int status)
{
    //设置是否对二维码反馈信息执行动作组响应
    Ennable_QR = status;
    QR.color = init_status;
}

/**********************************************************************
  * @Name    Set_QR_Target
  * @declaration : 设置二维码的目标颜色
  * @param   color: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void Set_QR_Target(int color)
{
    Ennable_QR = true;
    if (color != init_status)
    {

        if (color == 1)
            Target_Color = red;
        else if (color == 2)
            Target_Color = blue;
        else
            Target_Color = init_status;
    }
}

/**********************************************************************
  * @Name    DeInit_QRColor
  * @declaration :重新初始化二维码的颜色
  * @param   None
  * @retval   :无
  * @author  peach99CPP
***********************************************************************/
void DeInit_QRColor(void)
{
    QR.color = init_status;
}
