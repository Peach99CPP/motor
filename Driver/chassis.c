#include "chassis.h"
#include "motor.h"
#define TIME_PARAM 10
#define CHASSIS_RADIUS 18.0
#define MAX_SPEED 350.0
#include "track_bar_receive.h"
CHASSIS chassis;
float Radius_[5] = {0, \
                    30, \
                    30, \
                    30, \
                    30
                   };
float motor_target[5];
short time_count;
extern pid_paramer_t motor_param;
float control_val[5];


/**********************************************************************
  * @Name    get_chassis_speed
  * @declaration : get dir speed
  * @param   dir: [����/��] direct char
  * @retval   : specify direct speed
  * @author  peach99CPP
***********************************************************************/

float get_chassis_speed(char dir)
{
    if(dir == 'x' || dir == 'X')
    {
        return chassis.x_speed;
    }
    else if(dir == 'y' || dir == 'Y')
    {
        return chassis.y_speed;
    }
    else if(dir == 'w' || dir == 'W')
    {
        return chassis.w_speed;
    }
    else return 0;

}
/**********************************************************************
  * @Name    set_speed
  * @brief     ֱ���޸������̵��ٶ�ֵ
  * @param   x: [����/��]  x�����ٶ�
**			 y: [����/��]  y�����ٶ�
**			 w: [����/��]  w�����ٶ�
  * @retval  void
  * @author  peach99CPP
  * @Data    2021-08-06
***********************************************************************/
void set_speed(int x, int y, int w)
{
    if (chassis._switch)//ֻ�е����̵�ʹ�ܿ��ر���ʱ��������в���
    {
        chassis.x_speed = x;
        chassis.y_speed = y;
        chassis.w_speed = w;
    }
}

/************************************************************
*@name:change_switch_status
*@function:����ʹ�ܿ���
*@param:״̬��boolֵ
*@return:��
**************************************************************/
void change_switch_status(bool status)
{
    chassis._switch = status;
}

/**********************************************************************
  * @Name    speed_variation
  * @brief  �����ṩ�ٶ��޸ĵĽӿ�
  * @param   x_var: [����/��] x�����ٶȵĸı���
**			 y_var: [����/��] y�����ٶȵĸı���
**			 w_var: [����/��] w�����ٶȵĸı���
  * @retval  void
  * @author  peach99CPP
  * @Data    2021-08-06
***********************************************************************/

void speed_variation(float x_var, float y_var, float w_var)
{
    if(chassis._switch)
    {
        chassis.x_speed += x_var;
        chassis.y_speed += y_var;
        chassis.w_speed += w_var;
    }
}

/************************************************************
*@name:chassis_synthetic_control
*@function:���̵��ۺϿ��ƺ������������ֿ���
*@param:��
*@return:��
**************************************************************/
void chassis_synthetic_control(void)
{
    static int i, x_error = 0, y_error = 0;
    static float x, y, w, factor;
    static double max_val;
    if (chassis._switch == false ) return; //������̲���ʹ�ܣ���û�к�������

    if (++time_count == TIME_PARAM)
    {
        time_count = 0;
        y_error = track_pid_cal(&y_bar);
        x_error = track_pid_cal(&x_leftbar) + track_pid_cal(&x_rightbar);
    }
    max_val = 0;//�����ֵ���ݽ��г�ʼ��
    factor = 1;//�������ӳ�ʼ��

    x = chassis.x_speed;
    y = chassis.y_speed;
    w = chassis.w_speed;
    /***************************************
            1*************2
             *************
             *************
             *************
             *************
            3*************4
    ****************************************/
    motor_target[1] = 0.707 * y + 0.707 * x - Radius_[1] * w + y_error + x_error;
    motor_target[2] = -0.707 * y + 0.707 * x - Radius_[2] * w + y_error + x_error;
    motor_target[3] = 0.707 * y - 0.707 * x - Radius_[3] * w + y_error + x_error;
    motor_target[4] = 0.707 * y - 0.707 * x - Radius_[4] * w + y_error + x_error;

    //����һ���޷����������ⵥ���ٶȹ��ߵ��¿���Ч��������
    //



    for (i = 1; i <= 4; ++i) //�ҳ����ֵ
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    factor = (max_val > MAX_SPEED) ? MAX_SPEED / max_val : 1;
    if (max_val > MAX_SPEED)//���ֵ�Ƿ����ƣ����в�����ȷ�����ֵ���ڷ�Χ����ת�ٱ��� ����
    {
        factor = MAX_SPEED / max_val;
        for (i = 1; i < 4; ++i)
        {
            motor_target[i] *= factor;
        }

    }
    
    for (i = 1; i <= 4; ++i)
    {
        /*
        *�Ե�����б���
        *���Ȼ�ȡת���ڴ�ֵ
        *��ȡ����������
        *����PID���㺯���õ�����ֵ,�Է���ֵ��ʽ����
        *�ɼ���ֵ���Ƶ��
        */
        motor_data[i].expect = motor_target[i];
        motor_data[i].feedback = read_encoder(i);
        control_val[i] =  pid_control(&motor_data[i], &motor_param);
        set_motor(i, control_val[i]);
        if(motor_data[i].expect == motor_target[i] != 0)
        
        printf("%.2f",motor_data[i].feedback);
    }
    printf("\r\n");
    
//    //debug

//    motor_data[debug_motor_id].expect = motor_target[debug_motor_id];
//    motor_data[debug_motor_id].feedback = read_encoder(debug_motor_id);
//    control_val[debug_motor_id] =  pid_control(&motor_data[debug_motor_id], &motor_param);
//    set_motor(debug_motor_id, control_val[debug_motor_id]);
//    printf("%.2f     %.2f     %.2f\r\n",motor_data[debug_motor_id].feedback , motor_data[debug_motor_id].expect, motor_data[debug_motor_id].control_output);
}
