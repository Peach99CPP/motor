#include "openmv.h"
#include "servo.h"
#include "chassis.h"
#include "QR_code.h"
#include "read_status.h"

#include "uart_handle.h"

#define STOP_SIGNAL 0XAABB
short Handle_Flag = 0;
int mv_param;

int temp_ = 0;          //动作组计数器
short mv_stop_flag = 0; //判断MV舵控的工作状态
mvrec_t mv_rec;         // mv的结构体
mv_t MV =               //为结构体赋初值
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //初始化变量

volatile int disc_countval = 0, color_val = 0;

void Disc_Report(void)
{
    disc_countval++;
    printf("\n\t圆盘机拨球,当前目标%d,次数：%d \t\n", color_val, disc_countval);
    if ((disc_countval >= 8 && color_val == 0)|| (disc_countval >= 4 && color_val == 1))
    {
        disc_countval = 0;
        color_val++;
    }
}
int Get_DiscStatus(void)
{
    osDelay(5);         //防止整个系统被卡死 5ms为周期进行查询
    if (color_val == 1) //需要切换到黄球
    {
        return 1;
    }
    else if (color_val == 2)//表明此时都抓完了
    {
        return 2;
    }
    else
        return 0;
}

void Set_MV_Mode(bool mode)
{
    MV.enable_switch = mode;
}
bool Get_MV_Mode(void)
{
    return MV.enable_switch;
}
/**********************************************************************
 * @Name    cmd_encode
 * @declaration : 根据协议编码发送的内容
 * @param   event_id: [输入/出]  事件的类型
 **			 param: [输入/出]     参数，16位数字
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
    MV.mv_cmd[3] = l_byte;                                           //参数低8位
    MV.mv_cmd[4] = h_byte;                                           //参数高8位
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
    static int pn = 1; //正负标志符  后续会不断进行修改 为了避免性能浪费直接定义静态变量
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
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_Decode(void)
{
#define Catch_ 1
#define MVPID_THRESHOLD 10
#define pid_p 0.5
#define Ball_Signal 0x01
#define Rectangle_Signal 0x02
#define Ring_Signal 0x03
#define BAR_Signal 0x09
#define BAR_Action 30
#define MV_BACK 0x66

    if (Get_MV_Mode()) //只有此时mv是对信号响应，才进入下面的逻辑判断
    {
        if (Get_Servo_Flag()) //空闲，可以接收指令 此时openmv和舵控都准备好执行指令
        {
            //            Set_QR_Status(false); //关闭此时二维码那边的响应操作
            if (mv_rec.event == Ball_Signal)
            {
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Disable_ServoFlag();        //标记此时舵控正在运行过程中，本函数在传输舵控指令中也会被调用，此处只是为了增强记忆
                Enable_StopSignal();        //使能停车信号，让动作那边执行停车操作
                printf("要抓球\r\n");
                switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(Lowest, 1);
                    temp_++;
                    break;
                case MediumHeight:
                    Action_Gruop(Medium, 1);
                    temp_++;
                    break;
                case HighestHeight:
                    Action_Gruop(Highest, 1);
                    temp_++;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == Ring_Signal)
            {
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Enable_StopSignal();        //使能停车信号,此时进入停车状态，等待动作组执行完毕
                printf("发现圆环了\r\n");
                switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(LowestRing, 1);
                    temp_++;
                    break;
                case MediumHeight:
                    Action_Gruop(MediumRing, 1);
                    temp_++;
                    break;
                case HighestHeight:
                    Action_Gruop(HighestRing, 1);
                    temp_++;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == Rectangle_Signal)
            {
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Disable_ServoFlag();        //标记此时舵控正在运行过程中，本函数在传输舵控指令中也会被调用，此处只是为了增强记忆
                Enable_StopSignal();        //使能停车信号，让动作那边执行停车操作
                printf("扫到矩形\r\n");     //打印调试信息
                int height = Get_Height();
                switch (height) //根据当前高度来执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(Lowest, 1);
                    temp_ += 1;
                    break;
                case MediumHeight:
                    Action_Gruop(Medium, 1);
                    temp_ += 1;
                    break;
                case HighestHeight:
                    Action_Gruop(Highest, 1);
                    temp_ += 1;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == BAR_Signal) //自从修改爪子的形状之后  拨球的行为直接由舵控进行控制 因此需要对MV发回来的数据会进行解析处理
            {
                //主要区别在于此处不需要停车进行等待，所以只需清除舵控的标志位即可。
                Disable_ServoFlag(); //只需要清除舵机标志位即可 不需要停车
                Action_Gruop(BAR_Action, 1); //执行拨球动作组
                Disc_Report();
            }
            else if (mv_rec.event == MV_BACK)
            {
                if (mv_rec.param == 1)
                    printf("MV开始球\r\n");
                else if (mv_rec.param == 2)
                    printf("MV开始矩形\r\n");
                else if (mv_rec.param == 3)
                    printf("MV开始条形\r\n");
            }
        }
    }
}

/**********************************************************************
 * @Name    Get_Stop_Signal
 * @declaration :返回此时是否停止的信号
 * @param   None
 * @retval   : 是否应该停车，停车则为1
 * @author  peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @Name    Enable_StopSignal
 * @declaration :使能停车的标志位
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @Name    Disable_StopSignal
 * @declaration : 清除停车标志位
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
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
    MV_SendCmd(1, color);
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
void OpenMV_ChangeRoi(int roi)
{
    MV_SendCmd(11, roi);
    osDelay(100);
}
