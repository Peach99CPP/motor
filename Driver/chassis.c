#include "chassis.h"
#include "motor.h"
#define TIME_PARAM 10
#define CHASSIS_RADIUS 1.0
#define MAX_CHASSIS_SPEED 500
#define MAX_SPEED 370.0
#define MAX_CONTROL_VAL 9500
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "atk_imu.h"
double x_error = 0, y_error = 0, w_error = 0;
int i;
double speed_factor = 0, min_val;
CHASSIS_t chassis;
float Radius_[5] = {0,
                    1,
                    1,
                    1,
                    1};
float motor_target[5];
short time_count;
extern pid_paramer_t motor_param;
float control_val[5];

void w_speed_set(float w_speed)
{
    chassis.w_speed = w_speed;
}

/**********************************************************************
  * @Name    get_chassis_speed
  * @declaration : get dir speed
  * @param   dir: [����/��] direct char
  * @retval   : specify direct speed
  * @author  peach99CPP
***********************************************************************/

float get_chassis_speed(char dir)
{
    if (dir == 'x' || dir == 'X')
    {
        return chassis.x_speed;
    }
    else if (dir == 'y' || dir == 'Y')
    {
        return chassis.y_speed;
    }
    else if (dir == 'w' || dir == 'W')
    {
        return chassis.w_speed;
    }
    else
        return 0;
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
    if (chassis.enable_switch) //ֻ�е����̵�ʹ�ܿ��ر���ʱ��������в���
    {
        chassis.x_speed = x;
        chassis.y_speed = y;
        chassis.w_speed = w;
    }
}

/************************************************************
*@name:set_chassis_status
*@function:����ʹ�ܿ���
*@param:״̬��boolֵ
*@return:��
**************************************************************/
void set_chassis_status(bool status)
{
    chassis.enable_switch = status;
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
    if (chassis.enable_switch)
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
    static float x, y, w, factor;
    static double max_val;
    if (chassis.enable_switch == false)
        return; //������̲���ʹ�ܣ���û�к�������

    if (++time_count == TIME_PARAM)
    {
        time_count = 0;
        y_error = track_pid_cal(&y_bar);
        x_error = (track_pid_cal(&x_leftbar) - track_pid_cal(&x_rightbar)) / 2.0f;
    }

    max_val = 0; //�����ֵ���ݽ��г�ʼ��
    factor = 1;  //�������ӳ�ʼ��
    min_val = chassis.x_speed;
    if (min_val > y)
        min_val = chassis.y_speed;
    if (min_val > w)
        min_val = chassis.w_speed;
    if (min_val > 100)
    {
        speed_factor = min_val / 100.0;
    }
    else
        speed_factor = 1;

    if (Get_IMUStatus())
    {
        //�����ǿ���ʱ��Ѳ��ģʽ���Ѳ���ͨ��
        w_error = imu_correct_val();
        if(w_error < 20) w_error*=2;
        x = chassis.x_speed - y_error * speed_factor;
        y = chassis.y_speed - x_error * speed_factor;
        w = chassis.w_speed + w_error * speed_factor;

        /***************************************
                1*************2
                 *************
                 *************
                 *************
                 *************
                3*************4
        ****************************************/
        motor_target[1] = 0.707f * y + 0.707f * x - Radius_[1] * w;
        motor_target[2] = -0.707f * y + 0.707f * x - Radius_[2] * w;
        motor_target[3] = 0.707f * y - 0.707f * x - Radius_[3] * w;
        motor_target[4] = -0.707f * y - 0.707f * x - Radius_[4] * w;
    }
    else
    {
        //�����ǹر�״̬�µ�Ѳ��
        //�����ԣ�Ч�����ã�����
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
        //��������
        //����speed_factor��ԭ�������ڸ���ʱ��̬��ǿ����Ч��
        motor_target[1] = 0.707f * y + 0.707f * x - Radius_[1] * w + speed_factor * (y_error + x_error);
        motor_target[2] = -0.707f * y + 0.707f * x - Radius_[2] * w + speed_factor * (y_error + x_error);
        motor_target[3] = 0.707f * y - 0.707f * x - Radius_[3] * w + speed_factor * (y_error + x_error);
        motor_target[4] = -0.707f * y - 0.707f * x - Radius_[4] * w + speed_factor * (y_error + x_error);
    }

    //����һ���޷����������ⵥ���ٶȹ��ߵ��¿���Ч��������
    for (i = 1; i <= 4; ++i) //�ҳ����ֵ
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    factor = (max_val > MAX_SPEED) ? MAX_SPEED / max_val : 1;
    if (max_val > MAX_SPEED) //���ֵ�Ƿ����ƣ����в�����ȷ�����ֵ���ڷ�Χ����ת�ٱ��� ����
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
        control_val[i] = pid_control(&motor_data[i], &motor_param);
        if (control_val[i] > MAX_CONTROL_VAL)
            control_val[i] = MAX_CONTROL_VAL;
        if (control_val[i] < -MAX_CONTROL_VAL)
            control_val[i] = -MAX_CONTROL_VAL;
        set_motor(i, control_val[i]);
    }
    if (debug_motor_id >= 1)//ѭ��״̬��
    {
       printf("%.2f  %.2f %.2f %.2f ", motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect,\
       motor_data[debug_motor_id].control_output,motor_data[debug_motor_id].integrate\
       );
//        printf("%.2f  %.2f %.2f %.2f ",motor_data[1].feedback,motor_data[2].feedback,motor_data[3].feedback,motor_data[4].feedback);
        printf("\r\n");
    }
}
