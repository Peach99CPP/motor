#include "servo.h "
#include "uart_handle.h"
int Servo_Running = 1;
ServoControler_t servo_controler =
{
    .uart = &huart5,
    .current_index = 0,
    .cmd_buffer = {0},
    .rec_buffer = {0},
    .rec_index = 0
};



/**********************************************************************
  * @Name    Error_Report
  * @declaration : 进行错误报告
  * @param   type: [输入/出]  错误类型
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Error_Report(int type)
{
    if (type == 1)
        printf("角度值超限制\r\n");
    else if (type == 2)
        printf("动作组编号超限制\r\n");
    else
        printf("舵控发生未知错误\r\n");
}



/**********************************************************************
  * @Name    Cmd_Convert
  * @declaration : 将传入的参数编码成单个数字格式
  * @param   cmd: [输入/出]
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Cmd_Convert(int cmd)
{
    short temp_num[5] = {0}, index = 0;
    if (cmd == 0)//对为0的参数进行单独的处理
    {
        servo_controler.cmd_buffer[servo_controler.current_index++] = 0;
        return;
    }
    while (cmd != 0)//获取其数字的每一位，转化成字符
    {
        temp_num[index++] = cmd % 10 + '0';
        cmd /= 10;
    }
    index -= 1; //取得对应的下标
    while (index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

void Servo_Uart_Send(void)
{
    if (servo_controler.current_index == 0)
        return;
    //打包数据
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //发送
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //重新初始化结构体变量
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;
}
void Single_Control(int id, int control_mode, int angle, int time, int delay)
{
    if (id > 24 || control_mode > 3 || servo_controler.current_index != 0)
        return;
    //设置电机编号
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);

    /*根据控制模式设置要转到的角度
    mode 1: 直接原生角度，此时角度值应该在500-2500范围
    mode 2: 将输入的角度值转换到180舵机对应的角度值
    mode 3: 将输入的角度值转换到270舵机对应的角度值
    */
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'P';

    if (control_mode == 1)
    {
        if (angle > 2500 || angle < 500)
        {
            Error_Report(1);
            return;
        }
    }
    else if (control_mode == 2) //设置的是180度对应的角度
    {
        if (angle > 180 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 180.0) * angle; //转换到180度范围下对应的角度数值
    }
    else if (control_mode == 3)
    {
        if (angle > 270 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 270.0) * angle; //满角度为270度的舵机的指令
    }
    Cmd_Convert(angle);

    //设置执行时间
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //设置指令延迟
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert(delay);
    //通过串口发送
    Servo_Uart_Send();
}
void Action_Gruop(int id, int times)
{
    if (id > 255 || id < 0)//限制输入值，避免出错
    {
        Error_Report(2);
        return;
    }
    //设置动作组编号
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'G';
    Cmd_Convert(id);
    //设置重复次数
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'F';
    Cmd_Convert(times);
    //串口发送
    Servo_Uart_Send();
}
uint8_t mv_rec_flag = 0;
void Servo_RX_IRQ(void)
{
    if(__HAL_UART_GET_IT(servo_controler.uart, UART_IT_RXNE))
    {
        static uint8_t rec_data;
        rec_data = servo_controler.uart->Instance->RDR;
        servo_controler.rec_buffer[servo_controler.rec_index++] = rec_data;
        if(servo_controler.rec_index >= 2)
        {
            if(servo_controler.rec_buffer[0] == 'O' && servo_controler.rec_buffer[1] == 'K')
            {
                servo_controler.rec_index = 0;
                mv_rec_flag = 1;
            }
        }

    }
}
int  Get_Servo_Flag(void)
{
    if(mv_rec_flag == 1)
    {
        mv_rec_flag = 0;
        return true;
    }
    return false;
}
