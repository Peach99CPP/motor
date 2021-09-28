#include "track_bar_receive.h "
#include "usart.h"
#include "uart_handle.h"
#include "atk_imu.h"
#include <string.h>

extern int edge_status[3];
extern volatile uint32_t TIME_ISR_CNT;

#define START_BYTE 0xff
#define END_BYTE 0x99
#define MAX_LINE 3
#define BUFF_SIZE 9
#define LINE_DELAY (50 / 10)

int dma_count, times_counts = 0;
uint32_t y_time = 0, x_lefttime = 0, x_righttime = 0;

float track_weight[8] = {4, 3, 2, 1,
                         -1, -2, -3, -4};

uint8_t track_dma[MAX_LINE][BUFF_SIZE] = {0}, dma_trans_pos = 0; //DMA接收的数组

trackbar_t y_bar, x_leftbar, x_rightbar; //三个寻迹板
//初始化PID参数
pid_paramer_t track_pid_param = {
    .integrate_max = 50,
    .kp = 10,
    .ki = 0,
    .kd = 0,
    .control_output_limit = 300};

void Clear_Line(trackbar_t *bar)
{
    bar->line_flag = 0;
    bar->line_num = 0;
}

/**********************************************************************
  * @Name    set_track_pid
  * @declaration : 调试寻迹板的PID参数API
  * @param   kp: [输入/出] 放大10倍的p
**			 ki: [输入/出] 放大10倍的i
**			 kd: [输入/出] 放大10倍的d
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void set_track_pid(int kp, int ki, int kd)
{
    track_pid_param.kp = kp / 10.0;
    track_pid_param.ki = ki / 10.0;
    track_pid_param.kd = kd / 10.0;
}

/**********************************************************************
  * @Name    track_bar_init
  * @declaration : 对寻迹板需要的相关变量进行初始化的设置
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void track_bar_init(void) //相关的初始化函数
{
    dma_count = 0;
    dma_trans_pos = 0;
    //变量参数的初始化
    y_bar.id = forward_bar; //标注身份
    y_bar.line_num = 0;
    y_bar.num = 0;
    y_bar.line_num = 0;
    y_bar.data.expect = 0;  //循迹的目标值恒为0
    y_bar.if_switch = true; //使能

    x_leftbar.id = left_bar;
    x_leftbar.line_num = 0;
    x_leftbar.num = 0;
    x_leftbar.line_num = 0;
    x_leftbar.data.expect = 0;
    x_leftbar.if_switch = true;

    x_rightbar.id = right_bar;
    x_rightbar.line_num = 0;
    x_rightbar.num = 0;
    x_rightbar.line_num = 0;
    x_rightbar.data.expect = 0;
    x_rightbar.if_switch = true;
    HAL_UART_Receive_DMA(&TRACK_UART, track_dma[0], BUFF_SIZE);
}

/**********************************************************************
  * @Name    get_avaiable_pos
  * @declaration : 获取可用的数组下标
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
uint8_t get_avaiable_pos(void)
{
    //新版本
    for (uint8_t i = 0; i < MAX_LINE; ++i) //遍历数组index
    {
        if (track_dma[i][0] == START_BYTE && track_dma[i][BUFF_SIZE-1] == END_BYTE) //满足协议设计
        {
            return i;
        }
    }
    return 0XFF; //一个都找不到，返回特殊值，在调用时判断返回值非错误值可以减少出错的可能
}

/**********************************************************************
  * @Name    track_decode
  * @declaration :对DMA收到的数据进行解码计算，实现循迹各项功能的核心代码
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void track_decode(void)
{
    /***相关宏定义****/
#define EDGE_THRESHOLD 4 //在边缘数线模式下，几颗灯亮起时为有效计数
#define NUM_THRESHOLD 6  //非边缘线计算下，判断到达线的数量
#define MIN_NUM 2        //在压线之后，过线了才会计算一根线，根据灯的数量进行计数
#define EDGE_VAL 7       //边缘数线状态下的循迹读回来的值

    times_counts++;                   //总的处理次数，查看此 数据可以判断是否卡DMA
    dma_count--;                      //待处理数-1
    uint8_t pos = get_avaiable_pos(); //获取可用index
    uint8_t led_num = 0;
    float track_value = 0, temp_val; //相关的变量声明

    //下面的和检验看情况开启
    if ((uint8_t)(track_dma[pos][2] + track_dma[pos][4]+track_dma[pos][6]) == track_dma[pos][BUFF_SIZE-2]) //和校验
    {
        for (uint8_t bar_id = 1; bar_id <= BUFF_SIZE - 4; bar_id += 2)
        {
            track_value = 0;
            temp_val= 0;
            led_num = 0;
            for (uint8_t i = 0; i < 8; ++i)
            {
                temp_val = (bool)(((track_dma[pos][bar_id+1] << i) & 0x80)) * track_weight[i]; //根据灯亮与否及其权重得到反馈值
                if (temp_val != 0)
                    led_num++; //计算亮的灯数量
                track_value += temp_val;
            }
            switch (track_dma[pos][bar_id]) //判断寻迹板ID
            {
            case 1:
                y_bar.data.feedback = track_value; //赋值
                y_bar.num = led_num;               //得到灯的数量
                if (y_bar.num >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
                {
                    /*有两种情况，
                *一是当跑在非边缘时，此时灯的数量比较多
                *二是在边缘跑时，此时灯数量较少且需要加多重判断
                */
                    y_bar.line_flag = 1; //此时到线上
                }
                if (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL) //边缘数线的情况下，特殊处理
                    y_bar.data.feedback = 0;                                                               //要放在对线的判断之后；置0是为了防止此时发生偏移
                if (y_bar.line_flag && y_bar.num <= MIN_NUM)
                {
                    //使用此机制为了避免因停留在线上而导致线的数量一直重复计数
                    y_bar.line_flag = 0; //数线完成
                    y_bar.line_num++;    //线数目加一
                }
                break;
            case 2:
                x_leftbar.data.feedback = track_value;
                x_leftbar.num = led_num;

                if (x_leftbar.num >= NUM_THRESHOLD || (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL))
                {
                    x_leftbar.line_flag = 1; //标记到了线上
                }
                if (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL)
                    x_leftbar.data.feedback = 0;
                if (x_leftbar.line_flag && x_leftbar.num <= MIN_NUM) //避免因为在线上停留而导致的重复计数问题
                {
                    x_leftbar.line_flag = 0;
                    x_leftbar.line_num++;
                }
                break;
            case 3:
                x_rightbar.data.feedback = track_value;
                x_rightbar.num = led_num;
                if (x_rightbar.num >= NUM_THRESHOLD || (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL))
                {
                    x_rightbar.line_flag = 1;
                }
                if (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL)
                    x_leftbar.data.feedback = 0;
                if (x_rightbar.line_flag && x_rightbar.num <= MIN_NUM)
                {
                    x_rightbar.line_flag = 0;
                    x_rightbar.line_num++;
                }
                break;
            default: //啥也不干
                ;
            }
        }
        memset(track_dma[pos], 0, sizeof(track_dma[pos])); //把用到的清0，等待下次被填充
    }
}

/**********************************************************************
  * @Name    track_IT_handle
  * @declaration : 接收完成中断
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void track_IT_handle(void)
{
    uint8_t pos = get_avaiable_pos(); //通过获取这个检查是否有被填充进去
    if (pos != 0xff)
    {
        dma_count++;
        dma_trans_pos = ((dma_trans_pos + 1) >= MAX_LINE) ? 0 : dma_trans_pos + 1; //此接收器接收完成，。换到空闲接收区进行接收
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t *)track_dma[dma_trans_pos], BUFF_SIZE);
    }
    //否则就在原缓冲区继续接收
    else
    {
        memset(track_dma, 0, sizeof(track_dma)); //清空掉，作用不大，习惯为止
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t *)track_dma[dma_trans_pos], BUFF_SIZE);
    }
}

/**********************************************************************
  * @Name    HAL_UART_RxCpltCallback
  * @declaration : 自行修改的接收完成中断函数
  * @param   huart: [输入/出]  触发响应的串口
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &TRACK_UART) //寻迹板对应的串口，为了移植性，后续会用结构体
    {
        track_IT_handle();
    }
}

/**********************************************************************
  * @Name    track_pid_cal
  * @declaration : 寻迹板pid计算函数
  * @param   bar: [输入/出] 哪一个寻迹板
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
float track_pid_cal(trackbar_t *bar)
{
    if (bar->if_switch == true) //使能，计算pid值并进行返回
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0; //未使能，不做改变
}

/**********************************************************************
  * @Name    track_status
  * @declaration :设置寻迹板状态
  * @param   id: [输入/出]  方向。1为垂直 ，2为水平
**			 status: [输入/出]  开启或关闭
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void track_status(int id, int status)
{
    if (id == 1) //y方向
        y_bar.if_switch = status;
    else if (id == 2) //x方向
    {
        x_leftbar.if_switch = status;
        x_rightbar.if_switch = status;
    }
}
