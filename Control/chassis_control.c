
#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"

uint32_t time;
int x_line, y_line;
#define LINE_FACTOR 170
#define ENCODER_FACTOR 2
#define ENCODE_THRESHOLD 20
#define MAX_SPEED 600
#define MIN_SPEED 100
#define LINE_ERROR_ENCODER 150
static int dir, lines;

int edge_status[3] = {0};

osThreadId Line_Handle = NULL; //声明数线的任务句柄
void LineTask(void const * argument); //声明对应的变量

/**********************************************************************
  * @Name    move_slantly
  * @declaration : 倾斜起步
  * @param   dir: [输入/出]  方向 笛卡尔坐标系4大象限
**			 speed: [输入/出] 目标速度
**			 delay: [输入/出] 延迟的时间，用于飘逸到线上
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void move_slantly(int dir, int speed, uint16_t delay)
{
    int  x_factor, y_factor;
    switch(dir)
    {
    case 1:
        x_factor = 1, y_factor = 1;
        break;
    case 2:
        x_factor = -1, y_factor = 1;
        break;
    case 3:
        x_factor = -1, y_factor = -1;
        break;
    case 4:
        x_factor = 1, y_factor = -1;
        break;
    default:
        x_factor = 0, y_factor = 0;
    }
    set_speed(x_factor * speed, y_factor * speed, 0);
    osDelay(delay);
    set_speed(0, 0, 0);

}
/**********************************************************************
  * @Name    direct_move
  * @brief   通过循迹数线直线行进
  * @param   direct: [输入/出] 行进方向
  **		line_num: [输入/出]  要走的线数
  * @retval
  * @author  peach99CPP
  * @Data    2021-08-06
***********************************************************************/

void direct_move(int direct, int line_num, int edge_if)
{
    //使用任务创建的形式执行该函数
    if(direct == 1)
    {
        if(edge_if) edge_status[1] = edge_status[2] = 1;
        else        edge_status[1] = edge_status[2] = 0;
    }
    else if(direct == 2)
    {
        if(edge_if) edge_status[0] = 1;
        else     edge_status[0] = 0;
    }
    dir = direct;
    lines = line_num;
    osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 128);
    Line_Handle = osThreadCreate(osThread(line_task), NULL);
}
void LineTask(void const * argument)
{
    while(1)
    {
        static int speed_set = 0;
        static  short  error;
        if(dir == 1 && lines > 0)//水平向右
        {
            y_bar.if_switch  = false;
            x_leftbar.if_switch  = true;
            x_rightbar.if_switch = true;
            x_rightbar.line_num = 0;//把线的计数器清0
            do
            {
                error = lines - x_rightbar.line_num ;//计算还差几根线
                if(error == 0)
                {
                    set_speed(MIN_SPEED, 0, 0);//低速度走过去
                    osDelay(500);
                    goto EXIT_TASK;//任务结束
                }
                speed_set = LINE_FACTOR * error > MAX_SPEED ? MAX_SPEED : LINE_FACTOR * error;//普通情况下
                set_speed(speed_set * error, 0, 0);
                osDelay(5);
            }
            while( error >= 0);
        }
        else if(dir == 1 && lines < 0)
        {
            y_bar.if_switch  = false;
            x_leftbar.if_switch  = true;
            x_rightbar.if_switch = true;
            x_leftbar.line_num = 0;
            do
            {
                error = ABS(lines) - x_leftbar.line_num ;
                if(error == 0)
                {
                    set_speed(-MIN_SPEED, 0, 0);
                    osDelay(500);
                    goto EXIT_TASK;
                }
                speed_set = LINE_FACTOR * error > MAX_SPEED ? MAX_SPEED : LINE_FACTOR * error;
                set_speed(-speed_set, 0, 0);
                osDelay(5);
            }

            while( error >= 0);
        }
        else if(dir == 2 )
        {
            y_bar.if_switch  = true;
            x_leftbar.if_switch  = false;
            x_rightbar.if_switch = false;
            if(lines < 0) set_imu_angle(180);
            y_bar.line_num = 0;
            do
            {
                error = ABS(lines) - y_bar.line_num ;
                if(error == 0)
                {
                    set_speed(0, MIN_SPEED, 0);
                    while( x_leftbar.num == 0 && x_rightbar.num == 0)
                        osDelay(5);
                    goto EXIT_TASK;
                }
                speed_set = LINE_FACTOR * error > MAX_SPEED ? MAX_SPEED : LINE_FACTOR * error;
                set_speed(0, speed_set, 0);
                osDelay(5);
            }
            while(error >= 0 );
        }
    }
EXIT_TASK:
    set_speed(0, 0, 0);
    y_bar.if_switch  = true;
    x_leftbar.if_switch  = true;
    x_rightbar.if_switch = true;
    osDelay(2000);//开启路口矫正，开两秒就可以
    y_bar.if_switch  = false;
    x_leftbar.if_switch  = false;
    x_rightbar.if_switch = false;
    vTaskDelete(Line_Handle);
    Line_Handle = NULL;
}

/**********************************************************************
  * @Name    move_by_encoder
  * @功能说明  计算编码器值计算距离，面对侧向移动时需要添加转化
  * @param   val: [输入/出]  输入移动的值
  * @返回值
  * @author  peach99CPP
***********************************************************************/

void move_by_encoder(int  direct, int val)
{
    time = TIME_ISR_CNT;//获取系统时间
    double bias = 0, variation;//变量声明
    encoder_sum = 0;//将编码器累加值置0
    if(direct == 1)
    {
        y_bar.if_switch  = false;//关闭一侧的寻迹板
        x_leftbar.if_switch  = true;
        x_rightbar.if_switch = true;

        if(val < 0)//向左
        {
            while(ABS(val) > (encoder_sum / 1000))//未到达目标，除以1000是因为原生值过大
            {
                if( (TIME_ISR_CNT - time > 100) && ABS((val - (encoder_sum / 1000)) < ENCODE_THRESHOLD))goto EXIT_FUNC;//超时处理，避免卡死
                bias =  -(ABS(val) - (encoder_sum / 1000));//得到差值
                variation = bias * ENCODER_FACTOR;//计算得出输出值。P环
                variation = variation < - MAX_SPEED ? -MAX_SPEED : variation;//限幅
                set_speed(variation, 0, 0);//分配速度
                osDelay(1);//给任务调度内核切换的机会
            }
        }
        else
        {

            while(val > (encoder_sum / 1000))
            {
                if( (TIME_ISR_CNT - time > 100) && (val - (encoder_sum / 1000) < ENCODE_THRESHOLD))goto EXIT_FUNC;
                bias = val - (encoder_sum / 1000);
                variation = bias * ENCODER_FACTOR;
                variation = variation > MAX_SPEED ? MAX_SPEED : variation;
                set_speed(variation, 0, 0);
                osDelay(1);

            }
        }
    }
    else if(direct == 2)
    {
        y_bar.if_switch  = true;//关闭单侧的寻迹板
        x_leftbar.if_switch  = false;
        x_rightbar.if_switch = false;
        while(val > (encoder_sum / 1000))//同上
        {
            if( (TIME_ISR_CNT - time > 100) && (val - (encoder_sum / 1000) < ENCODE_THRESHOLD))
                goto EXIT_FUNC;
            bias = fabs(val - (encoder_sum / 1000));
            variation = bias * ENCODER_FACTOR;
            variation = variation > MAX_SPEED ? MAX_SPEED : variation;
            set_speed(0, variation, 0);
            osDelay(1);

        }
    }
EXIT_FUNC:
    set_speed(0, 0, 0);//停车
    //恢复寻迹板的使能状态
}
void car_shaking(int direct)
{
    while(y_bar.num == 0)
    {
        w_speed_set(40);
        osDelay(100);
        if(y_bar.num != 0) break;
        w_speed_set(40);
        osDelay(100);
    }
}
