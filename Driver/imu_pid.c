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
    //把传进来的角度限制在正负180范围
limit_label:
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
    if(ABS(angle) > 180) goto limit_label;//意义不大，但是避免出错
    return angle;
}
void set_imu_param(int p, int i, int d)
{
    //USMART调试设置参数
    imu_para.kp = (p/10.0);
    imu_para.ki = (i/100.0);
    imu_para.kd = (d/10.0);
}
void set_imu_status(int status)
{
    //改变陀螺仪的使能状态
    imu_switch = status;
}
//利用任务创建来实现自身角度的控制
void turn_angle(int rt_angle)
{
    if(imu_switch && last_finished )//使能了并且此时上一个任务运行完毕
    {
        target_angle = angle_limit(rt_angle + Yaw);//计算当前角度加上转的相对角度后需要转到的目标角度，转到正负180范围

        osThreadDef(imuturn_task, turn_rtangle, osPriorityHigh, 0, 256);//任务结构体的声明，有可能引起错误
        turn_taskHandle = osThreadCreate(osThread(imuturn_task), NULL);//CMSIS包装下的任务创建，兼容静态和动态
    }
}
void turn_rtangle(void const * argument)//任务实现函数
{
    last_finished = 0;//任务开始
    angle = target_angle;//获取目标角度
    imu_data.err = imu_data.last_err = imu_data.control_output = imu_data.integrate = 0;//初始化data结构体
    imu_data.expect = angle;//目标值
    old_time = TIME_ISR_CNT;//获取当前时间
    while(1)
    {
        if(TIME_ISR_CNT - old_time >MAX_CNT) //超时处理
            goto EXIT_TASK;
        
        init_angle = Yaw;//获取陀螺仪角度
        //对陀螺仪角度进行处理，把转弯幅度限制在180以内
        if((init_angle - angle) > 180 )  init_angle -= 360;
        if( (init_angle - angle) < -180) init_angle += 360;
        if( fabs(init_angle - angle) <= ERROR_THRESHOLD)//最终状态
        {
            
            int old_time = TIME_ISR_CNT;
            while(1)
            {
                imu_data.expect = angle;
                imu_data.feedback = init_angle;
                delta = pid_control(&imu_data, &imu_para);
                w_speed_set(delta);
                if((TIME_ISR_CNT - old_time) > DELAY_CNT) //到达设定时间退出
                    goto EXIT_TASK;
            }
        }
        //pid计算值
        imu_data.feedback = init_angle;
        delta = imu_pid_cal(&imu_data, &imu_para);
        w_speed_set(delta);
        vTaskDelay(10);
    }
EXIT_TASK:
    w_speed_set(0);//把w速度设置为0
    gyro_calibration();//矫正陀螺仪，作用不大
    last_finished = 1;//任务完成
    vTaskDelete(turn_taskHandle);//任务结束，删除任务
    turn_taskHandle = NULL;//任务句柄置为0
    return;

}
