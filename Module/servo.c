#include "servo.h "

ServoControler_t servo_controler =
{
    .uart = &huart5,
    .current_index = 0,
    .cmd_buffer = {0}
};


void Cmd_Convert(int  cmd)
{
     short temp_num[5] = {0}, index = 0;
    while(cmd != 0)
    {
        temp_num[index++] = cmd % 10+ '0';
        cmd /= 10;
    }
    index -= 1; //取得对应的下标
    while(index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

void Servo_Uart_Send(void)
{
    if(servo_controler.current_index == 0) return;
    //打包数据
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //发送
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //重新初始化结构体变量
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;

}
void Single_Control(int id, int control_mode, int angle, int  time, int delay)
{
    if(id > 24 || control_mode > 3 || servo_controler.current_index != 0) return;
    //设置电机编号
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);
    
    /*根据控制模式设置要转到的角度
    mode 1: 直接原生角度，此时角度值应该在500-2500范围
    mode 2: 将输入的角度值转换到180舵机对应的角度值
    mode 3: 将输入的角度值转换到270舵机对应的角度值
    */
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'P';
    
    if(control_mode == 1)
    {
        if(angle > 2500 || angle < 500 ) return;
    }
    else if(control_mode == 2) //设置的是180度对应的角度
        angle = 500 + (2000.0 / 180.0) * angle; //转换到180度范围下对应的角度数值
    else if( control_mode == 3)
        angle = 500 + (2000.0 / 270.0) * angle; //满角度为170度的舵机的指令
    Cmd_Convert(angle);
    //设置执行时间
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //设置指令延迟
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert( delay);
    //通过串口发送
    Servo_Uart_Send();
}
void Action_Gruop(int id, int  times)
{
    //设置动作组编号
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'G';
    Cmd_Convert( id);
    //设置重复次数
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'F';
    Cmd_Convert(times);
    //串口发送
    Servo_Uart_Send();
}
