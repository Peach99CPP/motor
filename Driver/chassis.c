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
  * @param   dir: [输入/出] direct char
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
  * @brief     直接修改器底盘的速度值
  * @param   x: [输入/出]  x方向速度
**			 y: [输入/出]  y方向速度
**			 w: [输入/出]  w方向速度
  * @retval  void
  * @author  peach99CPP
  * @Data    2021-08-06
***********************************************************************/
void set_speed(int x, int y, int w)
{
    if (chassis.enable_switch) //只有当底盘的使能开关被打开时才允许进行操作
    {
        chassis.x_speed = x;
        chassis.y_speed = y;
        chassis.w_speed = w;
    }
}

/************************************************************
*@name:set_chassis_status
*@function:底盘使能开关
*@param:状态，bool值
*@return:无
**************************************************************/
void set_chassis_status(bool status)
{
    chassis.enable_switch = status;
}

/**********************************************************************
  * @Name    speed_variation
  * @brief  对外提供速度修改的接口
  * @param   x_var: [输入/出] x方向速度的改变量
**			 y_var: [输入/出] y方向速度的改变量
**			 w_var: [输入/出] w方向速度的改变量
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
*@function:底盘的综合控制函数，包含多种控制
*@param:无
*@return:无
**************************************************************/
void chassis_synthetic_control(void)
{
    static float x, y, w, factor;
    static double max_val;
    if (chassis.enable_switch == false)
        return; //如果底盘不被使能，则没有后续操作

    if (++time_count == TIME_PARAM)
    {
        time_count = 0;
        y_error = track_pid_cal(&y_bar);
        x_error = (track_pid_cal(&x_leftbar) - track_pid_cal(&x_rightbar)) / 2.0f;
    }

    max_val = 0; //对最大值数据进行初始化
    factor = 1;  //倍率因子初始化
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
        //陀螺仪开启时的巡线模式，已测试通过
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
        //陀螺仪关闭状态下的巡线
        //经测试，效果不好，弃用
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
        //调试内容
        //乘上speed_factor的原因在于在高速时动态增强控制效果
        motor_target[1] = 0.707f * y + 0.707f * x - Radius_[1] * w + speed_factor * (y_error + x_error);
        motor_target[2] = -0.707f * y + 0.707f * x - Radius_[2] * w + speed_factor * (y_error + x_error);
        motor_target[3] = 0.707f * y - 0.707f * x - Radius_[3] * w + speed_factor * (y_error + x_error);
        motor_target[4] = -0.707f * y - 0.707f * x - Radius_[4] * w + speed_factor * (y_error + x_error);
    }

    //再来一个限幅操作，避免单边速度过高导致控制效果不理想
    for (i = 1; i <= 4; ++i) //找出最大值
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    factor = (max_val > MAX_SPEED) ? MAX_SPEED / max_val : 1;
    if (max_val > MAX_SPEED) //最大值是否超限制，进行操作，确保最大值仍在范围内且转速比例 不变
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
        *对电机进行遍历
        *首先获取转速期待值
        *读取编码器参数
        *传入PID计算函数得到计算值,以返回值形式传参
        *由计算值控制电机
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
    if (debug_motor_id >= 1)//循迹状态下
    {
       printf("%.2f  %.2f %.2f %.2f ", motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect,\
       motor_data[debug_motor_id].control_output,motor_data[debug_motor_id].integrate\
       );
//        printf("%.2f  %.2f %.2f %.2f ",motor_data[1].feedback,motor_data[2].feedback,motor_data[3].feedback,motor_data[4].feedback);
        printf("\r\n");
    }
}
