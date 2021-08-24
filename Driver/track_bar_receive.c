#include "track_bar_receive.h "
#include "usart.h"
#define START_BYTE  0xff
#define END_BYTE    0x99
#define MAX_LINE 6
#define BUFF_SIZE 5
int dma_count;
float track_weight[8] = { -4, -3, -2, 0, \
                          0, 2, 3, 4 \
                        };
uint8_t track_dma[MAX_LINE][BUFF_SIZE] = {0}, dma_trans_pos = 0;
trackbar_t y_bar, x_leftbar, x_rightbar;
pid_paramer_t track_pid_param = { \
                            .integrate_max = 20,
                            .kp = 10,
                            .ki = 1,
                            .kd = 0,
                            .control_output_limit = 40
                          };



void track_bar_init(void)
{
    dma_trans_pos =0;
    
    y_bar.id = forward_bar;
    y_bar.data.expect =0;
    y_bar.if_switch = true;

    x_leftbar.id = left_bar;
    x_leftbar.data.expect =0;
    x_leftbar.if_switch = true;

    x_rightbar.id = right_bar;
    x_rightbar.data.expect =0;
    x_rightbar.if_switch = true;

    HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma, BUFF_SIZE);
}
uint8_t get_idle_pos(void)
{
    uint8_t i = 0;
    for(uint8_t i =0;i< MAX_LINE;++i)
    {
        if(track_dma[i][0] == START_BYTE && track_dma[i][4] == END_BYTE)
        {
            return i;
        }
    }
    return 0XFF;
}
void track_decode(void)
{
    dma_count--;
    uint8_t pos = get_idle_pos();
    float track_value = 0,temp_val;
//    if((uint8_t)(track_dma[pos][0] + track_dma[pos][1] + track_dma[pos][2] )== track_dma[pos][3]) //ºÍÐ£Ñé
//    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            temp_val= (bool)(((track_dma[pos][2] << i)&0x80)) * track_weight[i];
            track_value+=temp_val;
        }
        switch (track_dma[pos][1])
        {
        case 1:
            y_bar.data.feedback = track_value;
            break;
        case 2:
            x_leftbar.data.feedback = track_value;
            break;
        case 3:
            x_rightbar.data.feedback= track_value;
            break;
        default:
            ;
        }
   memset(track_dma[pos],0,sizeof(track_dma[pos]));
    
}
void track_IT_handle(void)
{
    uint8_t pos = get_idle_pos();
    if(pos !=0xff)
    {
    dma_count++;

    dma_trans_pos = ((dma_trans_pos+1) >= MAX_LINE)?0:dma_trans_pos+1;
    HAL_UART_Receive_DMA(&TRACK_UART,(uint8_t*)track_dma[dma_trans_pos],BUFF_SIZE);
    }
    else HAL_UART_Receive_DMA(&TRACK_UART,(uint8_t*)track_dma[dma_trans_pos],BUFF_SIZE);
    
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        track_IT_handle();
    }
}
float track_pid_cal(trackbar_t * bar)
{
    if(bar->if_switch == true)
    {
        return pid_control(&bar->data,&track_pid_param);
    }
    return 0;
}
