#include "openmv.h"
#include "servo.h"
#include "chassis.h"
#include "uart_handle.h"
#include "read_status.h"

#define STOP_SIGNAL 0XAABB
int mv_param;
static short mv_stop_flag = 0;
mvrec_t mv_rec;
mv_t MV =
{
    .mv_uart = &huart4,
    .mv_cmd = {0},
    .rec_buffer = {0},
    .rec_len = 0,
    .RX_Status = 0
}; //初始化变量

/**********************************************************************
  * @Name    cmd_encode
  * @declaration : 根据协议编码发送的内容
  * @param   event_id: [输入/出]  时间的类型
**			 param: [输入/出] 参数
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void cmd_encode(const uint8_t event_id, int param)
{
    static uint8_t pos_flag;
    if (param > 0)
        pos_flag = 1;
    else
    {
        pos_flag = 2;
        param *= -1;
    }

    uint8_t h_byte, l_byte; //获取参数的高8位和低8位
    h_byte = (param >> 8);
    l_byte = (param & 0xff);
    //定义通讯协议
    MV.mv_cmd[0] = START_BYTE; //帧头
    MV.mv_cmd[1] = event_id;   //触发的事件id
    MV.mv_cmd[2] = pos_flag;
    MV.mv_cmd[3] = l_byte;                                           //参数高8位
    MV.mv_cmd[4] = h_byte;                                           //参数低8位
    MV.mv_cmd[5] = (uint8_t)(event_id + pos_flag + h_byte + l_byte); //和校验
    MV.mv_cmd[6] = END_BYTE;                                         //帧尾
}
void MV_SendCmd(const uint8_t event_id, const int param)
{
    cmd_encode(event_id, param);                                 //根据获得的参数编码cmd数组
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //将cmd发送出去
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //将cmd数组重新初始化
}

/**********************************************************************
  * @Name    MV_IRQ
  * @declaration :  openmv通讯的中断处理函数
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_IRQ(void)
{
    if (MV.RX_Status < 2) //未接收完成
    {
        uint8_t rec_data = MV.mv_uart->Instance->RDR; //读取本次接收的值
        if (rec_data == START_BYTE && MV.RX_Status == 0)
        {
            MV.RX_Status = 1; //读到帧头，做标记，直接退出
            MV.rec_len = 0;
            return;
        }
        if (MV.RX_Status == 1) //收到帧头后
        {
            if (rec_data == END_BYTE && MV.rec_len == 5) //帧头帧尾都接收到了,为了避免PID传输过程中出错，需要指定接收长度
            {
                MV.RX_Status = 2; //标记接收完成
                MV_rec_decode();  //对接收到的内容进行解码操作
            }
            else
            {
                //未收到帧尾，此时是数据内容
                MV.rec_buffer[MV.rec_len++] = rec_data; //存入数组
                if (MV.rec_len == MAX_REC_SIZE)
                {
                    MV.RX_Status = 0; //防止因为出错导致卡死
                    MV.rec_len = 0;
                    memset(MV.rec_buffer, 0, sizeof(MV.rec_buffer));
                }
            }
        }
    }
}

/**********************************************************************
  * @Name    MV_rec_decode
  * @declaration : 判断接收完成后，对接收的内容进行解码
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_rec_decode(void)
{
    static int pn = 1; //正负标志符
    if (MV.rec_buffer[0] + MV.rec_buffer[1] + MV.rec_buffer[2] + MV.rec_buffer[3] == MV.rec_buffer[4])
    {
        //根据参数内容对参数进行处理
        if (MV.rec_buffer[1] == 1)
            pn = 1;
        else
            pn = -1;
        mv_rec.event = MV.rec_buffer[0];
        mv_rec.param = (MV.rec_buffer[2] + (MV.rec_buffer[3] << 8)) * pn;
        MV_Decode();
        MV.rec_len = 0;
        MV.RX_Status = 0;
    }
    //处理完之后记得重新初始化结构体中的rec_len和RX_status变量，避免出错
    ;
}

/****上面是底层实现，下面是上层的应用****/


/**********************************************************************
  * @Name    MV_PID
  * @declaration :让MV返回PID信号
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_PID(void)
{
    MV_SendCmd(2, 0);
}

/**********************************************************************
  * @Name    MV_SendOK
  * @declaration :向openmv发送OK信号
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_SendOK(void)
{
    MV_SendCmd(4, 0);
}

/**********************************************************************
  * @Name    MV_Decode
  * @declaration :根据自己定义的参数含义执行命令
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_Decode(void)
{
#define Catch_ 1
#define MVPID_THRESHOLD 10
#define pid_p 0.5
#define Ball_Signal 0x04
    if (mv_rec.event == Ball_Signal)
    {
        set_speed(0, 0, 0);
        mv_stop_flag = 1;
        osDelay(300);
        printf("收到了球的信息,Paarmter=%d\r\n",mv_rec.param);    //todo：调试模式下进行使用，用于测试是否收到了消息
        if (mv_rec.param == ladder_type) //阶梯平台三种高度
        {
            switch (Get_Height())
            {
            case 1:
                Action_Gruop(Lowest, 1);
                break;
            case 2:
                Action_Gruop(Highest, 1);
                break;
            case 3:
                Action_Gruop(Medium, 1);
            default:
                ;
            }
        }
        else if (mv_rec.param == bar_type)
        {
            Action_Gruop(Bar, 1);
        }
        osDelay(2000);
    }
}



/**********************************************************************
  * @Name    Get_Stop_Signal
  * @declaration :返回此时是否停止的信号
  * @param   None
  * @retval   : 是否应该停车
  * @author  peach99CPP
***********************************************************************/
int Get_Stop_Signal(void)
{
    if (mv_stop_flag)
    {
        mv_stop_flag = 0;
        return true;
    }
    else
        return false;
}



/**********************************************************************
  * @Name    MV_Start
  * @declaration : Mv开始响应命令
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_Start(void)
{
    MV_SendCmd(0, 0);
}

/**********************************************************************
  * @Name    MV_Scan_High
  * @declaration :让MV扫描阶梯平台的高层
  * @param   color: [输入/出] 要抓的球的颜色
  * @retval   :无
  * @author  peach99CPP
***********************************************************************/
void MV_Scan_High(mvcolor_t color)
{
    MV_SendCmd(2, color);
}



/**********************************************************************
  * @Name    MV_Scan_Low
  * @declaration : 让MV扫描阶梯平台的中层及下层
  * @param   color: [输入/出]  要抓的球的颜色
  * @retval   :无
  * @author  peach99CPP
***********************************************************************/
void MV_Scan_Low(mvcolor_t color)
{
    MV_SendCmd(2, color);
}



/**********************************************************************
  * @Name    MV_Scan_Bar
  * @declaration :让MV扫描条形平台
  * @param   color: [输入/出]  要抓的球的颜色
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_Scan_Bar(mvcolor_t color)
{
    MV_SendCmd(3, color);
}



/**********************************************************************
  * @Name    MV_Stop
  * @declaration : 向MV发送停止信号
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_Stop(void)
{
    MV_SendCmd(4, 0);
}
