
#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "uart_handle.h"

int x_line, y_line;
#define LINE_FACTOR 0.5
#define ENCODER_FACTOR 0.5
#define MAX_SET_SPEED 250



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
    int x_factor, y_factor;
    if(speed>MAX_SET_SPEED)
    {
        printf("over speed\r\n");
        return ;
    }
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

void direct_move(direct_t direct, int line_num)
{
    if(direct == hor) //水平即左右
    {
        float x_var;
        while(ABS(line_num) > ABS(x_line))
        {
            x_var = LINE_FACTOR * (line_num - x_line);
            speed_variation(x_var, 0, 0);
        }
    }
    else if(direct == ver)//
    {
        float y_var;
        while(ABS(line_num) > ABS(y_line))
        {
            y_var = LINE_FACTOR * (line_num - y_line);
            speed_variation(0, y_var, 0);
        }
    }
    else return ;
}



/**********************************************************************
  * @Name    move_by_encoder
  * @功能说明  计算编码器值计算距离，面对侧向移动时需要添加转化
  * @param   val: [输入/出]  输入移动的值
  * @返回值
  * @author  peach99CPP
***********************************************************************/

void move_by_encoder(direct_t direct, int val)
{
    double bias = 0, variation;
    encoder_sum = 0;
    if(direct == hor)
    {

        while(val > encoder_sum)
        {
            bias = fabs(val - encoder_sum);
            variation = bias * ENCODER_FACTOR;
            speed_variation(variation, 0, 0);

        }
    }
    else if(direct == ver)
    {
        while(val > encoder_sum)
        {
            bias = fabs(val - encoder_sum);
            variation = bias * ENCODER_FACTOR;
            speed_variation(0, variation, 0);

        }
    }
}
