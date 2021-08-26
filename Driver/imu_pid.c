#include "imu_pid.h"
#include "chassis.h"
#include "time_cnt.h"
#include "imu.h"

#define ERROR_THRESHOLD 5
#define DELAY_TIME  500
#define DELAY_CNT   (DELAY_TIME/10)
#define MAX_TIME 30000
#define MAX_CNT  (MAX_TIME/10)
pid_data_t imu_data;
pid_paramer_t imu_para =
{
    .kp = 1.5,
    .ki = 0.05,
    .kd = 0.1,
    .integrate_max = 60,
    .control_output_limit = 250
};

osThreadId turn_taskHandle;//任务句柄
extern volatile uint32_t TIME_ISR_CNT;
int  imu_switch = 1, init_ = 0, last_finished = 1;

volatile float delta, init_angle, angle, target_angle;
uint32_t old_time;

void turn_rtangle(void const * argument);//函数声明

float  angle_limit(float  angle)
{
limit_label:
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
    if(ABS(angle) > 180) goto limit_label;
    return angle;
}
void set_imu_param(int p, int i, int d)
{
    imu_para.kp = (p/10.0);
    imu_para.ki = (i/100.0);
    imu_para.kd = (d/10.0);
}
void set_imu_status(int status)
{
    imu_switch = status;
}
//利用任务创建来实现自身角度的控制
void turn_angle(int rt_angle)
{
    if(imu_switch && last_finished )//使能了并且此时上一个任务运行完毕
    {
        target_angle = angle_limit(rt_angle + Yaw);

        osThreadDef(imuturn_task, turn_rtangle, osPriorityHigh, 0, 256);
        turn_taskHandle = osThreadCreate(osThread(imuturn_task), NULL);
    }
}
void turn_rtangle(void const * argument)
{
    last_finished = 0;
    angle = target_angle;
    imu_data.err = imu_data.last_err = imu_data.control_output = imu_data.integrate = 0;//初始化data结构体
    imu_data.expect = angle;
    old_time = TIME_ISR_CNT;
    while(1)
    {
        if(TIME_ISR_CNT - old_time >MAX_CNT) 
            goto EXIT_TASK;
        init_angle = Yaw;
        if((init_angle - angle) > 180 )  init_angle -= 360;
        if( (init_angle - angle) < -180) init_angle += 360;
        if( fabs(init_angle - angle) <= ERROR_THRESHOLD)
        {
            
            int old_time = TIME_ISR_CNT;
            while(1)
            {
                imu_data.expect = angle;
                imu_data.feedback = init_angle;
                delta = pid_control(&imu_data, &imu_para);
                w_speed_set(delta);
                if((TIME_ISR_CNT - old_time) > DELAY_CNT) 
                    goto EXIT_TASK;
            }
        }
        imu_data.feedback = init_angle;
        delta = imu_pid_cal(&imu_data, &imu_para);
        w_speed_set(delta);
        vTaskDelay(10);
    }
EXIT_TASK:
    w_speed_set(0);
    gyro_calibration();
    last_finished = 1;
    vTaskDelete(turn_taskHandle);
    turn_taskHandle = NULL;
    return;

}
