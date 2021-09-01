#include "track_bar_receive.h "
#include "usart.h"
#include "uart_handle.h"
#define START_BYTE  0xff
#define END_BYTE    0x99
#define MAX_LINE 6
#define BUFF_SIZE 5
int dma_count,times_counts=0;
float track_weight[8] = { -4, -3, -2, 0, \
                          0, 2, 3, 4 \
                        };
uint8_t track_dma[MAX_LINE][BUFF_SIZE] = {0}, dma_trans_pos = 0;
trackbar_t y_bar, x_leftbar, x_rightbar;
pid_paramer_t track_pid_param = { \
                                  .integrate_max = 50,
                                  .kp = 10,
                                  .ki = 0,
                                  .kd = 0,
                                  .control_output_limit = 200
                                };

void set_track_pid(int kp,int ki,int kd)
{
    track_pid_param.kp=  kp/10.0;
    track_pid_param.ki=  ki/10.0;
    track_pid_param.kd=  kd/10.0;
}

void track_bar_init(void)//相关的初始化函数
{
    dma_count=0;
    dma_trans_pos = 0;
    //变量参数的初始化
    y_bar.id = forward_bar;//标注身份
    y_bar.data.expect = 0;//循迹的目标值恒为0
    y_bar.if_switch = true;//使能

    x_leftbar.id = left_bar;
    x_leftbar.data.expect = 0;
    x_leftbar.if_switch = true;

    x_rightbar.id = right_bar;
    x_rightbar.data.expect = 0;
    x_rightbar.if_switch = true;
    //开启DMA接收，此处应确保main中DMA初始化函数在串口初始化前运行
    HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma, BUFF_SIZE);
}
uint8_t get_idle_pos(void)
{
    //获取此时有数据的数组index
    uint8_t i = 0;
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

    float track_value = 0, temp_val; //相关的变量声明
    //下面的和检验看情况开启
    if((uint8_t)( track_dma[pos][1] + track_dma[pos][2] ) == track_dma[pos][3]) //和校验
    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            temp_val = (bool)(((track_dma[pos][2] << i) & 0x80)) * track_weight[i];
            track_value += temp_val;
        }
        switch (track_dma[pos][1])//判断寻迹板ID
        {
        case 1:
            y_bar.data.feedback = track_value;
            break;
        case 2:
            x_leftbar.data.feedback = track_value;
            break;
        case 3:
            x_rightbar.data.feedback = track_value;
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
    else HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)//寻迹板对应的串口，为了移植性，后续会用结构体
    {
        track_IT_handle();
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
