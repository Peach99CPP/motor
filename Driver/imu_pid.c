#include "imu_pid.h"
#include "chassis.h"
#include "time_cnt.h"

#define ERROR_THRESHOLD 5

pid_data_t imu_data;
pid_paramer_t imu_para;

osThreadId turn_taskHandle;//任务句柄
void turn_rtangle(void const * argument);//函数声明
extern volatile uint32_t TIME_ISR_CNT;

float  angle_limit(float  angle)
{
limit_label:
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
    if(ABS(angle) > 180) goto limit_label;
    return angle;
}
int  imu_switch = 0;
//利用任务创建来实现自身角度的控制
void turn_angle(float rt_angle)
{
    if(imu_switch && turn_taskHandle == NULL )//使能了并且此时上一个任务运行完毕
    {
        static float target_angle ;

        target_angle = angle_limit(rt_angle);
        
        osThreadDef(imuturn_task, turn_rtangle, osPriorityHigh, 0, 256);
        turn_taskHandle = osThreadCreate(osThread(imuturn_task), &target_angle);
    }
}
void turn_rtangle(void const * argument)
{
    float angle = *((float *)argument );//获取参数
    float delta, init_angle;
    if(fabs((init_angle - angle)) < 1) goto EXIT_TASK;
    imu_data.err = imu_data.last_err = 0;
    while(1)
    {
        init_angle = Yaw;
        if((init_angle - angle) > 180 )  init_angle -= 360;
        if( (init_angle - angle) < -180) init_angle += 360;
        if( (init_angle - angle) <= ERROR_THRESHOLD) goto  EXIT_TASK;
        imu_data.expect = angle;
        imu_data.feedback = init_angle;
        delta = pid_control(&imu_data, &imu_para);
        speed_variation(0, 0, delta);
        vTaskDelay(10);
    }
EXIT_TASK:
    vTaskDelete(NULL);
    turn_taskHandle = NULL;

}
