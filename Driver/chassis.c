#include "chassis.h"
#define TIME_PARAM 10
#define CHASSIS_RADIUS 18.0
#define MAX_SPEED 180.0
CHASSIS chassis;
float motor_target[5];
short time_count;
/************************************************************
*@name:set_speed
*@function:ֱ�Ӷ�Ŀ���ٶ�ֵ�����޸�
*@param:����������ٶ�
*@return:��
**************************************************************/
void set_speed (int x,int y)
{
    chassis.x_speed=x;
    chassis.y_speed=y;
}
/************************************************************
*@name:change_switch_stage
*@function:����ʹ�ܿ���
*@param:״̬��boolֵ
*@return:��
**************************************************************/
void change_switch_stage(bool status)
{
    chassis._switch=status;
}
/************************************************************
*@name:chassis_synthetic_control
*@function:���̵��ۺϿ��ƺ������������ֿ���
*@param:��
*@return:��
**************************************************************/
void chassis_synthetic_control(void)
{
    int  i;
    float x,y,w,max_val,factor;
    if(chassis._switch == false) return;//������̲���ʹ�ܣ���û�к�������
    
    if(++time_count == TIME_PARAM) 
    {
        time_count = 0;
        //������ˢһЩ�����Ǻ�Ѱ����PID����ֵ.���޸�ֵ������chassis��x y�ٶ���
    }
    max_val = 0;//�����ֵ���ݽ��г�ʼ��
    factor = 1;//�������ӳ�ʼ��
    
    x = chassis.x_speed;
    y = chassis.y_speed;
    w = chassis.w_speed; 
    /***************************************
    * motor1  ���Ͻ�
    * motor2  ���½�
    * motor3  ���½�
    * motor4  ���Ͻ�
    * �����Ͻ���ʱ����תһȦ������4����� 1 2 3 4
    ****************************************/
    
    motor_target[1] = -0.707 * x - 0.707 * y + CHASSIS_RADIUS * w;
    motor_target[2] = 0.707 * x - 0.707 * y + CHASSIS_RADIUS * w;
    motor_target[3] = 0.707 * x + 0.707 * y + CHASSIS_RADIUS * w;
    motor_target[4]= -0.707 * x + 0.707 * y + CHASSIS_RADIUS * w;
    
    //����һ���޷����������ⵥ���ٶȹ��ߵ��¿���Ч��������
    for(i=1;i<=4;++i)//�ҳ����ֵ
    {
        if(motor_target[i]>max_val)
            max_val = motor_target[i];
    }
    factor = ( max_val > MAX_SPEED ) ? MAX_SPEED / max_val : 1;
    if(max_val > MAX_SPEED)
    {
        factor = MAX_SPEED / max_val;
        for(i = 1;i < 4; ++ i)
        {
            motor_target[i] *= factor;
        }
        
    }
    for(i=1;i<=4;++i)
    {   /*
        *�Ե�����б���
        *���Ȼ�ȡת���ڴ�ֵ
        *��ȡ����������
        *����PID���㺯���õ�����ֵ,�Է���ֵ��ʽ����
        *�ɼ���ֵ���Ƶ��
        */
        motor_controler[i].expect=motor_target[i];
        motor_controler[i].feedback=read_encoder(i);
        set_motor(i,pid_control(&motor_controler[i]));
    }
   
    
}