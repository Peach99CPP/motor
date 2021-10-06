#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "motor.h"
#include "imu_pid.h"
uint32_t time;

#define LINE_FACTOR 150

#define MAX_SPEED 500
#define MIN_SPEED 120
#define LINE_ERROR_ENCODER 150

static int dir, lines, en_dir, en_val;
static int count_line_status = 1, encodermove_status = 1;
int edge_status[3] = {0};
double bias = 0, variation; //变量声明

osThreadId Line_Handle = NULL;       //声明数线的任务句�???
void LineTask(void const *argument); //声明对应的变�???

osThreadId Encoder_Handle = NULL; //声明数线的任务句�???
void EncoderTask(void const *argument);

/**********************************************************************
  * @Name    move_slantly
  * @declaration : 倾斜起步
  * @param   dir: [输入/出]  方向 笛卡尔坐标系4大象�???
**			 speed: [输入/出] 目标速度
**			 delay: [输入/出] 延迟的时间，用于飘逸到线上
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void move_slantly(int dir, int speed, uint16_t delay)
{
    set_imu_status(true); //开启陀螺仪保证角度稳定
    int x_factor, y_factor;
    switch (dir)
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
    osDelay(500);
}
/**********************************************************************
  * @Name    direct_move
  * @brief   通过循迹数线直线行进
  * @param   direct: [输入/出] 行进方向
  **		line_num: [输入/出]  要走的线�???
  * @retval
  * @author  peach99CPP
  * @Data    2021-08-06
***********************************************************************/

void direct_move(int direct, int line_num, int edge_if,int imu_if)
{
    static int delay_time;
    if (count_line_status) //确保上一个任务完成的情况下，再执行下一个任�???
    {
START_LINE:
        set_imu_status(imu_if);
        //使用任务创建的形式执行该函数
        if (direct == 1)
        {
            if (edge_if)
                edge_status[1] = edge_status[2] = 1;
            else
                edge_status[1] = edge_status[2] = 0;
        }
        else if (direct == 2)
        {
            if (edge_if)
                edge_status[0] = 1;
            else
                edge_status[0] = 0;
        }
        count_line_status = 0; //外部获知任务完成与否的依�???

        dir = direct;
        lines = line_num;
        osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 256);
        Line_Handle = osThreadCreate(osThread(line_task), NULL);
    }
    else
    {
        delay_time = 0;
        while (!count_line_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 20)
                return;
        }
        goto START_LINE;
    }
}
void LineTask(void const *argument)
{
    y_bar.if_switch = true;
    x_leftbar.if_switch = true;
    x_rightbar.if_switch = true;
    while (1)
    {
        static int speed_set = 0;
        static short error;
        if (dir == 1 && lines > 0) //水平向右
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
            Clear_Line(&x_leftbar);
            do
            {
                error = lines - x_leftbar.line_num; //计算还差几根�???
                if (error == 0)
                {
                    set_speed(MIN_SPEED, 0, 0); //低速度走过�???
                    while ((x_leftbar.num == 0 && x_rightbar.num == 0) || (y_bar.num == 0))
                        osDelay(5); //y方向寻迹板有灯，x其中一个方向有�???
                    goto EXIT_TASK; //任务结束
                }
                speed_set = Limit_Speed(LINE_FACTOR * error); //普通情况下
                set_speed(speed_set * error, 0, 0);
                osDelay(5);
            }
            while (error >= 0);
        }
        else if (dir == 1 && lines < 0)
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
            x_rightbar.line_num = 0;
            Clear_Line(&x_rightbar);
            do
            {
                error = ABS(lines) - x_rightbar.line_num;
                if (error == 0)
                {
                    set_speed(-MIN_SPEED, 0, 0);
                    while ((x_leftbar.num == 0 && x_rightbar.num == 0) || (y_bar.num == 0))
                        osDelay(5); //y方向寻迹板有灯，x其中一个方向有�???
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(-speed_set, 0, 0);
                osDelay(5);
            }

            while (error >= 0);
        }
        else if (dir == 2)
        {
            y_bar.if_switch = true;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
            Clear_Line(&y_bar);
            if (lines < 0)
            {
                turn_angle(1, 180); //以相对角度转�???180�???
                lines *= -1;
                while (!get_turn_status())
                    osDelay(1); //确保此时转弯完成，才进入下一阶段的行�???
            }
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    set_speed(0, MIN_SPEED, 0);
                    while (x_leftbar.num == 0 && x_rightbar.num == 0)
                        osDelay(5);
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(0, speed_set, 0);
                osDelay(5);
            }
            while (error >= 0);
        }
    }
EXIT_TASK:
    set_speed(0, 0, 0);
    count_line_status = 1;
    y_bar.if_switch = true;
    x_leftbar.if_switch = true;
    x_rightbar.if_switch = true;
    osDelay(2000); //开启路口矫正，开两秒就可�???
    y_bar.if_switch = false;
    x_leftbar.if_switch = false;
    x_rightbar.if_switch = false;
    vTaskDelete(Line_Handle);
    Line_Handle = NULL;
}

/**********************************************************************
  * @Name    move_by_encoder
  * @功能说明  计算编码器值计算距离，面对侧向移动时需要添加转�???
  * @param   val: [输入/出]  输入移动的�?
  * @返回�???
  * @author  peach99CPP
***********************************************************************/
/*9.14修改记录，将函数运行方式改为进程模式，尝试解决相同移动数值的问题*/
void move_by_encoder(int direct, int val)
{
    static int encoder_delay;
    if (encodermove_status) //上一个任务运行结束，才可以开始运行下一个任务，避免出错
    {
START_ENCODER:
        set_imu_status(true); //确保陀螺仪开启，试验性，不确定要不要

        en_dir = direct; //将参数传递给全局变量�???
        en_val = val;

        encoder_sum = 0; //将编码器累加值置0
        //开启任�???
        osThreadDef(encodermove, EncoderTask, osPriorityRealtime, 0, 256); //任务优先级给到最高，确保及时响应
        Encoder_Handle = osThreadCreate(osThread(encodermove), NULL);
        //任务结束标志
        encodermove_status = 0;
    }
    else
    {
        encoder_delay = 0;
        while (!encodermove_status)
        {
            encoder_delay++;
            osDelay(100);
            if (encoder_delay >= 20)
                return;
        }
        goto START_ENCODER;
    }
}
void EncoderTask(void const *argument)
{
#define ENOCDER_DIVIDE_FACTOR 10
#define ENCODE_THRESHOLD 2
#define ENCODER_FACTOR 2
    clear_motor_data();
    time = TIME_ISR_CNT; //获取系统时间
    if (en_dir == 1)
    {
        y_bar.if_switch = false; //关闭一侧的寻迹�???
        x_leftbar.if_switch = true;
        x_rightbar.if_switch = true;

        if (en_val < 0) //向左
        {
            en_val *= -1;
            while (1) //未到达目�???
            {
                if ((TIME_ISR_CNT - time > 50) && ABS((en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                    goto Encoder_Exit;                                       //超时处理，避免卡�???
                bias = -ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)); //得到差�?
                variation = bias * ENCODER_FACTOR;                           //计算得出输出值。P�???
                variation = Limit_Speed(variation);                          //分配最低速度，避免卡�???
                set_speed(variation, 0, 0);                                  //分配速度

                osDelay(5); //给任务调度内核切换的机会
            }
        }
        else
        {
            //向右为正
            while (1)
            {
                if ((TIME_ISR_CNT - time > 50) && ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR) < ENCODE_THRESHOLD))
                    goto Encoder_Exit;
                bias = ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
                variation = bias * ENCODER_FACTOR;
                variation = Limit_Speed(variation);
                set_speed(variation, 0, 0);
                osDelay(5);
            }
        }
    }
    else if (en_dir == 2)
    {
        static int pn;
        y_bar.if_switch = true; //关闭单侧的寻迹板
        x_leftbar.if_switch = false;
        x_rightbar.if_switch = false;
        if (en_val < 0)
        {
            pn = -1;      //标记其为负数
            en_val *= -1; //绝对值化
        }
        else
            pn = 1;
        while (1) //同上
        {
            if ((TIME_ISR_CNT - time > 50) && ABS((en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR) < ENCODE_THRESHOLD)))
                goto Encoder_Exit;
            bias = fabs(ABS(en_val) - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
            variation = bias * ENCODER_FACTOR;
            variation = Limit_Speed(variation);
            set_speed(0, variation * pn, 0);
            osDelay(5);
        }
    }
Encoder_Exit:
    set_speed(0, 0, 0);          //停车
    encodermove_status = 1;      //标记结束�???
    vTaskDelete(Encoder_Handle); //从任务列表中将其移出
    Encoder_Handle = NULL;       //将指针指向空
}

/**********************************************************************
  * @Name    car_shaking
  * @declaration : 一个试验性的测试功能，可能用于卸货时确保货物被甩下来
  * @param   direct: [输入/出] 往哪一个方向进行摇�???
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void car_shaking(int direct)
{
    while (y_bar.num == 0)
    {
        w_speed_set(40);
        osDelay(100);
        if (y_bar.num != 0)
            break;
        w_speed_set(-40);
        osDelay(100);
    }
}

/**********************************************************************
  * @Name    get_count_line_status
  * @declaration : 外部获知数线运行的完成情况的接口
  * @param   None
  * @retval   : 是否运行完成
  * @author  peach99CPP
***********************************************************************/
int get_count_line_status(void)
{
    return count_line_status;
}
/**********************************************************************
  * @Name    get_enocdermove_status
  * @declaration : 外部获知靠编码器运行的完成情况的接口
  * @param   None
  * @retval   :  是否运行结束
  * @author  peach99CPP
***********************************************************************/
int get_enocdermove_status(void)
{
    return encodermove_status;
}



/**********************************************************************
  * @Name    Limit_Speed
  * @declaration :对输入的角度值进行双向限幅后输出
  * @param   speed: [输入/出] 待限幅的速度�??
  * @retval   :  限幅过后的速度值，既不太高又不太低
  * @author  peach99CPP
***********************************************************************/
int Limit_Speed(int speed)
{
    if (speed > 0)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
        if (speed < MIN_SPEED)
            speed = MAX_SPEED;
    }
    else
    {
        if (speed < -MAX_SPEED)
            speed = -MAX_SPEED;
        if (speed > -MIN_SPEED)
            speed = -MIN_SPEED;
    }
    return speed;
}
void Comfirm_Online(int dir)
{
    #define LOW_SPEED_TO_CONFIRM 40
    if(dir== 1 )
    {
        set_speed(-LOW_SPEED_TO_CONFIRM,0,0);
        while(Get_Trcker_Num(&y_bar) <= 1)osDelay(10);
    }
    else if(dir == 2)
    {
        set_speed(LOW_SPEED_TO_CONFIRM,0,0);
        while(Get_Trcker_Num(&y_bar) <= 1) osDelay(10);
    }
    set_speed(0,0,0);
    track_status(1,1);
    osDelay(1000);
}
