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
    index -= 1; //ȡ�ö�Ӧ���±�
    while(index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

void Servo_Uart_Send(void)
{
    if(servo_controler.current_index == 0) return;
    //�������
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //����
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //���³�ʼ���ṹ�����
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;

}
void Single_Control(int id, int control_mode, int angle, int  time, int delay)
{
    if(id > 24 || control_mode > 3 || servo_controler.current_index != 0) return;
    //���õ�����
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);
    
    /*���ݿ���ģʽ����Ҫת���ĽǶ�
    mode 1: ֱ��ԭ���Ƕȣ���ʱ�Ƕ�ֵӦ����500-2500��Χ
    mode 2: ������ĽǶ�ֵת����180�����Ӧ�ĽǶ�ֵ
    mode 3: ������ĽǶ�ֵת����270�����Ӧ�ĽǶ�ֵ
    */
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'P';
    
    if(control_mode == 1)
    {
        if(angle > 2500 || angle < 500 ) return;
    }
    else if(control_mode == 2) //���õ���180�ȶ�Ӧ�ĽǶ�
        angle = 500 + (2000.0 / 180.0) * angle; //ת����180�ȷ�Χ�¶�Ӧ�ĽǶ���ֵ
    else if( control_mode == 3)
        angle = 500 + (2000.0 / 270.0) * angle; //���Ƕ�Ϊ170�ȵĶ����ָ��
    Cmd_Convert(angle);
    //����ִ��ʱ��
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //����ָ���ӳ�
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert( delay);
    //ͨ�����ڷ���
    Servo_Uart_Send();
}
void Action_Gruop(int id, int  times)
{
    //���ö�������
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'G';
    Cmd_Convert( id);
    //�����ظ�����
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'F';
    Cmd_Convert(times);
    //���ڷ���
    Servo_Uart_Send();
}
