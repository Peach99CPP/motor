/* ************************************************************
  *
  * FileName   : imu_pid.c
  * Version    : v1.0
  * Author     : peach99CPP
  * Date       : 2021-09-07
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
  ******************************************************************************
 */
#include "imu_pid.h"
#include "atk_imu.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

pid_data_t imu_data, anglekeep_data;
pid_paramer_t imu_para =
{
    .kp = 9,
    .ki = 0,
    .kd = 0,
    .integrate_max = 60,
    .control_output_limit = 300
};

extern ATK_IMU_t  imu;
float delta;



float imu_correct_val(void)
{
    static float now_angle;
    if(! imu.switch_ ) return 0;
    else
    {
        imu_data.expect = imu.target_angle;
        now_angle = imu.get_angle();
        if(now_angle - imu.target_angle > 180 ) now_angle -= 360;
        if(now_angle - imu.target_angle < -180 ) now_angle += 360;
        imu_data.feedback = now_angle;
        delta = pos_pid_cal(&imu_data, &imu_para);
        return delta;

    }
    return 0;
}

void set_imu_angle(int  angle)
{
    imu.switch_ = 1;
    if(angle == 180 )
    {
        imu.target_angle = 179.5;
        return;

    }
    imu.target_angle = angle_limit(angle);

}
/**********************************************************************
  * @Name    angle_limit
  * @declaration :
  * @param   angle: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

float  angle_limit(float  angle)
{
    //把传进来的角度限制在正负180范围
limit_label:
    while(angle > 180) angle -= 360;
    while(angle <= -180) angle += 360;
    if(ABS(angle) > 180) goto limit_label;//意义不大，但是避免出错
    return angle;
}

void set_imu_param(int p, int i, int d)
{
    //USMART调试设置参数
    imu_para.kp = (p / 10.0);
    imu_para.ki = (i / 100.0);
    imu_para.kd = (d / 10.0);
}

void set_imu_status(int status)
{
    //改变陀螺仪的使能状态
    imu.switch_ = status;
}

