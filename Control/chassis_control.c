#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "motor.h"
#include "imu_pid.h"
#include "Wait_BackInf.h"
#include "uart_handle.h"
uint32_t time;

#define LINE_FACTOR 160

#define MAX_SPEED 500
#define MIN_SPEED 120
#define LINE_ERROR_ENCODER 150

int dir, lines, en_dir, en_val;
int count_line_status = 1, encodermove_status = 1;
int edge_status[3] = {0};
double bias = 0, variation;

osThreadId Line_Handle = NULL;
void LineTask(void const *argument);

osThreadId Encoder_Handle = NULL;
void EncoderTask(void const *argument);

void move_slantly(int dir, int speed, uint16_t delay)
{
    set_imu_status(true);
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

/**
 * @description:  开启一个任务用于数线
 * @param {int} direct 方向：竖向为2 横向为1
 * @param {int} line_num 数几条线
 * @param {int} edge_if 是否是在边缘（对阈值情况进行特殊处理）
 * @param {int} imu_if  是否开启陀螺仪
 * @return {*}
 */    
void direct_move(int direct, int line_num, int edge_if, int imu_if)
{
    static int delay_time;
    if (count_line_status)
    {
    START_LINE:
        set_imu_status(imu_if);

        Clear_Line(&y_bar);
        Clear_Line(&x_rightbar);
        Clear_Line(&x_leftbar);
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
        count_line_status = 0;

        dir = direct;
        lines = line_num;
        osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 256);
        Line_Handle = osThreadCreate(osThread(line_task), NULL);
    }
    else
    {
        //当上一次数线任务进行中时，此时不会再度调起数线任务，而是进行等待。
        //当等待时间过长时，直接取消本次数线运动。
        delay_time = 0;
        while (!count_line_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 20)
            {
                printf("\n数线任务等待时间过长, 已经退出\n");
                return;
            }
        }
        goto START_LINE;
    }
}
void LineTask(void const *argument)
{
    int if_need_zero = 0; //

    y_bar.if_switch = true;
    x_leftbar.if_switch = true;
    x_rightbar.if_switch = true;
    while (1)
    {
        static int speed_set = 0;
        static short error;
        if (dir == 1 && lines > 0)
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
            Clear_Line(&x_rightbar);
            do
            {
                error = lines - x_rightbar.line_num;
                if (error == 0)
                {
                    set_speed(MIN_SPEED, 0, 0);
                    while (y_bar.num == 0)
                    {
                        osDelay(5);
                    }
                    Comfirm_Online(2);
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(speed_set , 0, 0);//修正了此前速度变化过快的错误 是因为此处等价于平方效应
                osDelay(5);
            } while (error >= 0);
        }
        else if (dir == 1 && lines < 0)
        {

            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
            lines *= -1;
            Clear_Line(&x_leftbar);
            do
            {
                error = lines - x_leftbar.line_num;
                if (error == 0)
                {
                    set_speed(-MIN_SPEED, 0, 0);
                    while (y_bar.num == 0)
                    {
                        osDelay(5);
                    }
                    Comfirm_Online(1);
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(-speed_set, 0, 0);
                osDelay(5);
            } while (error >= 0);
        }
        else if (dir == 2)
        {

            y_bar.if_switch = true;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
            if (lines < 0)
            {
                // todo
                if_need_zero = 1;
                Turn_angle(1, 180, 0);
                lines *= -1;
            }
            Clear_Line(&y_bar);
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    set_speed(0, MIN_SPEED, 0);
                    while (x_leftbar.num == 0 && x_rightbar.num == 0)
                        osDelay(5);
                    Comfirm_Online(3);
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(0, speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
    }
EXIT_TASK:
    set_speed(0, 0, 0);
    //在下面的两个分支中都会执行Delay
    if (if_need_zero)
        Turn_angle(1, 180, 1);
    else
    {
        y_bar.if_switch = true;
        x_leftbar.if_switch = true;
        x_rightbar.if_switch = true;
        osDelay(2000);
        y_bar.if_switch = false;
        x_leftbar.if_switch = false;
        x_rightbar.if_switch = false;
    }
    count_line_status = 1;
    vTaskDelete(NULL);
    Line_Handle = NULL;
}
/**
 * @name: move_by_encoder
 * @brief: 用于开启编码器运行任务的函数
 * @param {int} direct 方向 竖直为2  水平为1 
 * @param {int} val 参照平面坐标系的XY轴正负
 * @return {*} 无
 */
void move_by_encoder(int direct, int val)
{
    static int encoder_delay;
    if (encodermove_status)
    {
    START_ENCODER:
        set_imu_status(true);

        en_dir = direct;
        en_val = val;

        encoder_sum = 0;

        osThreadDef(encodermove, EncoderTask, osPriorityRealtime, 0, 256);
        Encoder_Handle = osThreadCreate(osThread(encodermove), NULL);

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
#define ENOCDER_DIVIDE_FACTOR 50.0
#define ENCODE_THRESHOLD 0.5
#define ENCODER_FACTOR 8
    clear_motor_data();
    time = TIME_ISR_CNT;
    if (en_dir == 1)
    {
        y_bar.if_switch = false;
        x_leftbar.if_switch = true;
        x_rightbar.if_switch = true;

        if (en_val < 0)
        {
            en_val *= -1;
            while (1)
            {
                if ((TIME_ISR_CNT - time > 50) && ((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                    goto Encoder_Exit;
                bias = -((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
                variation = bias * ENCODER_FACTOR;
                variation = Limit_Speed(variation);
                set_speed(variation, 0, 0);

                osDelay(5);
            }
        }
        else
        {

            while (1)
            {
                if ((TIME_ISR_CNT - time > 50) && ((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                    goto Encoder_Exit;
                bias = ((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
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
        y_bar.if_switch = true;
        x_leftbar.if_switch = false;
        x_rightbar.if_switch = false;
        if (en_val < 0)
        {
            pn = -1;
            en_val *= -1;
        }
        else
            pn = 1;
        while (1)
        {
            if ((TIME_ISR_CNT - time > 50) && ((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                goto Encoder_Exit;
            bias = fabs((double)en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
            variation = bias * ENCODER_FACTOR;
            variation = Limit_Speed(variation);
            set_speed(0, variation * pn, 0);
            osDelay(5);
        }
    }
Encoder_Exit:
    set_speed(0, 0, 0);
    encodermove_status = 1;
    vTaskDelete(NULL);
    Encoder_Handle = NULL;
}

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

int get_count_line_status(void)
{
    return count_line_status;
}

int get_enocdermove_status(void)
{
    return encodermove_status;
}

int Limit_Speed(int speed)
{
    if (speed > 0)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
        if (speed < MIN_SPEED)
            speed = MIN_SPEED;
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
#define LOW_SPEED_TO_CONFIRM 80
#define Track_Time 1000
    track_status(1, 1);
    track_status(2, 1);
    if (dir == 1)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(-LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 2)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 3)
    {
        if (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
        {
            set_speed(0, LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    track_status(1, 0);
    track_status(1, 0);
}
void Wait_OKInf(int type, long wait_time)
{
    long temp_time = 100;
    if (type == 1)
    {
        osDelay(temp_time);
        while (!get_count_line_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
    }
    else if (type == 2)
    {
        osDelay(temp_time);
        while (!get_enocdermove_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
    }
    osDelay(200);
}

/**
 * @name:  inte_move
 * @brief: 运动的集成
 * @param {int} type  类型 1 为数线 2为输编码器
 * @param {int} dir   方向 1为横向 2为竖向
 * @param {int} val    数  需要的数值 分别对应数线的数值和编码器数值
 * @param {int} edge    在数线模式下使用 用于区分是否为边缘状态
 * @param {int} imu_if  过程中是否开启陀螺仪
 * @param {long} wait_time  最长的等待时间
 * @return {*}              返回是否运行成功  当返回false代表有错误产生
 */
bool inte_move(int type, int dir, int val, int edge, int imu_if, long wait_time)
{
    if (type == 1)
    {
        direct_move(dir, val, edge, imu_if);
        Wait_OKInf(type, wait_time);
        return true;
    }
    else if (type == 2)
    {
        move_by_encoder(dir, val);
        Wait_OKInf(type, wait_time);
        return true;
    }
    printf("传入的参数类型出错 \n");
    return false;
}
