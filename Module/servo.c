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
  * @declaration : ���д��󱨸�
  * @param   type: [����/��]  ��������
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Error_Report(int type)
{
    if (type == 1)
        printf("�Ƕ�ֵ������\r\n");
    else if (type == 2)
        printf("�������ų�����\r\n");
    else
        printf("��ط���δ֪����\r\n");
}



/**********************************************************************
  * @Name    Cmd_Convert
  * @declaration : ������Ĳ�������ɵ������ָ�ʽ
  * @param   cmd: [����/��]
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Cmd_Convert(int cmd)
{
    short temp_num[5] = {0}, index = 0;
    if (cmd == 0)//��Ϊ0�Ĳ������е����Ĵ���
    {
        servo_controler.cmd_buffer[servo_controler.current_index++] = 0;
        return;
    }
    while (cmd != 0)//��ȡ�����ֵ�ÿһλ��ת�����ַ�
    {
        temp_num[index++] = cmd % 10 + '0';
        cmd /= 10;
    }
    index -= 1; //ȡ�ö�Ӧ���±�
    while (index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

void Servo_Uart_Send(void)
{
    if (servo_controler.current_index == 0)
        return;
    //�������
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //����
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //���³�ʼ���ṹ�����
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;
}
void Single_Control(int id, int control_mode, int angle, int time, int delay)
{
    if (id > 24 || control_mode > 3 || servo_controler.current_index != 0)
        return;
    //���õ�����
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);

    /*���ݿ���ģʽ����Ҫת���ĽǶ�
    mode 1: ֱ��ԭ���Ƕȣ���ʱ�Ƕ�ֵӦ����500-2500��Χ
    mode 2: ������ĽǶ�ֵת����180�����Ӧ�ĽǶ�ֵ
    mode 3: ������ĽǶ�ֵת����270�����Ӧ�ĽǶ�ֵ
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
    else if (control_mode == 2) //���õ���180�ȶ�Ӧ�ĽǶ�
    {
        if (angle > 180 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 180.0) * angle; //ת����180�ȷ�Χ�¶�Ӧ�ĽǶ���ֵ
    }
    else if (control_mode == 3)
    {
        if (angle > 270 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 270.0) * angle; //���Ƕ�Ϊ270�ȵĶ����ָ��
    }
    Cmd_Convert(angle);

    //����ִ��ʱ��
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //����ָ���ӳ�
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert(delay);
    //ͨ�����ڷ���
    Servo_Uart_Send();
}
void Action_Gruop(int id, int times)
{
    if (id > 255 || id < 0)//��������ֵ���������
    {
        Error_Report(2);
        return;
    }
    //���ö�������
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'G';
    Cmd_Convert(id);
    //�����ظ�����
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'F';
    Cmd_Convert(times);
    //���ڷ���
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
