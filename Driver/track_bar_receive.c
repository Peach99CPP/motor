#include "track_bar_receive.h "
#include "usart.h"
#include "uart_handle.h"
#include  "atk_imu.h"
#include <string.h>


extern int edge_status[3];
extern volatile uint32_t TIME_ISR_CNT;

#define START_BYTE  0xff
#define END_BYTE    0x99
#define MAX_LINE 3
#define BUFF_SIZE 5
#define NUM_THRESHOLD 6
#define MIN_NUM 3
#define LINE_DELAY (50/10)

int dma_count, times_counts = 0;
uint32_t y_time = 0, x_lefttime = 0, x_righttime = 0;

float track_weight[8] = { 4, 3, 2, 0, \
                          0, -2, -3, -4 \
                        };
uint8_t track_dma[MAX_LINE][BUFF_SIZE] = {0}, dma_trans_pos = 0;
trackbar_t y_bar, x_leftbar, x_rightbar;
pid_paramer_t track_pid_param = { \
                                  .integrate_max = 50,
                                  .kp = 10,
                                  .ki = 0,
                                  .kd = 0,
                                  .control_output_limit = 300
                                };

void set_track_pid(int kp, int ki, int kd)
{
    track_pid_param.kp =  kp / 10.0;
    track_pid_param.ki =  ki / 10.0;
    track_pid_param.kd =  kd / 10.0;
}

void track_bar_init(void)//相关的初始化函数
{
    dma_count = 0;
    dma_trans_pos = 0;
    //变量参数的初始化
    y_bar.id = forward_bar;//标注身份
    y_bar.line_num = 0;
    y_bar.num = 0;
    y_bar.line_num = 0;
    y_bar.data.expect = 0;//循迹的目标值恒为0
    y_bar.if_switch = true;//使能

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
    //开启DMA接收，此处应确保main中DMA初始化函数在串口初始化前运行
    HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma, BUFF_SIZE);
}
uint8_t get_idle_pos(void)
{
    //获取此时有数据的数组index
    for(uint8_t i = 0; i < MAX_LINE; ++i) //遍历数组index
    {
        if(track_dma[i][0] == START_BYTE && track_dma[i][4] == END_BYTE)//满足协议设计
        {
            return i;
        }
    }
    return 0XFF;//一个都找不到，返回特殊值，在调用时判断返回值非错误值可以减少出错的可能
}
void track_decode(void)
{
    times_counts++;
    dma_count--;//待处理数-1
    uint8_t pos = get_idle_pos();//获取可用index
    uint8_t led_num = 0;
    float track_value = 0, temp_val; //相关的变量声明
    //下面的和检验看情况开启
    if((uint8_t)( track_dma[pos][1] + track_dma[pos][2] ) == track_dma[pos][3]) //和校验
    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            temp_val = (bool)(((track_dma[pos][2] << i) & 0x80)) * track_weight[i];
            if( temp_val != 0) led_num++;
            track_value += temp_val;
        }
        switch (track_dma[pos][1])//判断寻迹板ID
        {
        case 1:
            y_bar.data.feedback = track_value;
            y_bar.num = led_num;
            if(edge_status[0] && y_bar.num >= 5 )
                y_bar.data.feedback = 0;
            if(y_bar.num  >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= 5 ))
            {
                y_time = TIME_ISR_CNT;
                y_bar.line_flag  = 1;
            }
            if(y_bar.line_flag && y_bar.num <= MIN_NUM && (TIME_ISR_CNT - y_time) > LINE_DELAY )
            {
                y_time = TIME_ISR_CNT;
                y_bar.line_flag = 0;
                y_bar.line_num ++;
            }
            break;
        case 2:
            x_leftbar.data.feedback = track_value;
            x_leftbar.num = led_num;
            if(edge_status[0] && x_leftbar.num >= 5)
                x_leftbar.data.feedback = 0;
            if(x_leftbar.num >= NUM_THRESHOLD || (edge_status[0] && x_leftbar.num >= 5))
            {
                x_lefttime = TIME_ISR_CNT;
                x_leftbar.line_flag  = 1; //标记到了线上
            }
            if(x_leftbar.line_flag && x_leftbar.num <= MIN_NUM && TIME_ISR_CNT - x_lefttime > LINE_DELAY ) //避免因为在线上停留而导致的重复计数问题
            {
                x_lefttime = TIME_ISR_CNT;
                x_leftbar.line_flag = 0;
                x_leftbar.line_num ++;
            }
            break;
        case 3:
            x_rightbar.data.feedback = track_value;
            x_rightbar.num = led_num;
            if(edge_status[0] && x_rightbar.num >= 5 )
                x_rightbar.data.feedback = 0;
            if( x_rightbar.num >= NUM_THRESHOLD || (edge_status[0] && x_rightbar.num >= 5 ))
            {
                x_righttime = TIME_ISR_CNT;
                x_rightbar.line_flag  = 1;
            }
            if(x_rightbar.line_flag && x_rightbar.num <= MIN_NUM && TIME_ISR_CNT - x_righttime > LINE_DELAY )
            {
                x_righttime = TIME_ISR_CNT;
                x_rightbar.line_flag = 0;
                x_rightbar.line_num ++;
            }
            break;
        default://啥也不干
            ;
        }
        memset(track_dma[pos], 0, sizeof(track_dma[pos])); //把用到的清0，等待下次被填充
    }

}
void track_IT_handle(void)
{
    uint8_t pos = get_idle_pos();//通过获取这个检查是否有被填充进去
    if(pos != 0xff)
    {
        dma_count++;
        dma_trans_pos = ((dma_trans_pos + 1) >= MAX_LINE) ? 0 : dma_trans_pos + 1; //此接收器接收完成，。换到空闲接收区进行接收
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);
    }
    //否则就在原缓冲区继续接收
    else 
    {
        memset(memset(track_dma,0,sizeof(track_dma)),0,sizeof(memset(track_dma,0,sizeof(track_dma))));
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);
        
    }
    
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)//寻迹板对应的串口，为了移植性，后续会用结构体
    {
        track_IT_handle();
    }
    else if(huart ==  imu.imu_uart) 
    {
        IMU_IRQ();
    }
    
}
float track_pid_cal(trackbar_t * bar)
{
    if(bar->if_switch == true)//使能，计算pid值并进行返回
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0;//未使能，不做改变
}
void track_status(int id, int status)
{
    if(id == 1 )
        y_bar.if_switch = status;
    else if(id == 2)
    {
        x_leftbar.if_switch = status;
        x_rightbar.if_switch = status;
    }
}
