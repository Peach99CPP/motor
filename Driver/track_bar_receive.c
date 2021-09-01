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

void track_bar_init(void)//��صĳ�ʼ������
{
    dma_count=0;
    dma_trans_pos = 0;
    //���������ĳ�ʼ��
    y_bar.id = forward_bar;//��ע���
    y_bar.data.expect = 0;//ѭ����Ŀ��ֵ��Ϊ0
    y_bar.if_switch = true;//ʹ��

    x_leftbar.id = left_bar;
    x_leftbar.data.expect = 0;
    x_leftbar.if_switch = true;

    x_rightbar.id = right_bar;
    x_rightbar.data.expect = 0;
    x_rightbar.if_switch = true;
    //����DMA���գ��˴�Ӧȷ��main��DMA��ʼ�������ڴ��ڳ�ʼ��ǰ����
    HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma, BUFF_SIZE);
}
uint8_t get_idle_pos(void)
{
    //��ȡ��ʱ�����ݵ�����index
    uint8_t i = 0;
    for(uint8_t i = 0; i < MAX_LINE; ++i) //��������index
    {
        if(track_dma[i][0] == START_BYTE && track_dma[i][4] == END_BYTE)//����Э�����
        {
            return i;
        }
    }
    return 0XFF;//һ�����Ҳ�������������ֵ���ڵ���ʱ�жϷ���ֵ�Ǵ���ֵ���Լ��ٳ���Ŀ���
}
void track_decode(void)
{
    times_counts++;
    dma_count--;//��������-1
    uint8_t pos = get_idle_pos();//��ȡ����index

    float track_value = 0, temp_val; //��صı�������
    //����ĺͼ��鿴�������
    if((uint8_t)( track_dma[pos][1] + track_dma[pos][2] ) == track_dma[pos][3]) //��У��
    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            temp_val = (bool)(((track_dma[pos][2] << i) & 0x80)) * track_weight[i];
            track_value += temp_val;
        }
        switch (track_dma[pos][1])//�ж�Ѱ����ID
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
        default://ɶҲ����
            ;
        }
        memset(track_dma[pos], 0, sizeof(track_dma[pos])); //���õ�����0���ȴ��´α����
    }

}
void track_IT_handle(void)
{
    uint8_t pos = get_idle_pos();//ͨ����ȡ�������Ƿ��б�����ȥ
    if(pos != 0xff)
    {
        dma_count++;
        dma_trans_pos = ((dma_trans_pos + 1) >= MAX_LINE) ? 0 : dma_trans_pos + 1; //�˽�����������ɣ����������н��������н���
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);
    }
    //�������ԭ��������������
    else HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)//Ѱ�����Ӧ�Ĵ��ڣ�Ϊ����ֲ�ԣ��������ýṹ��
    {
        track_IT_handle();
    }
}
float track_pid_cal(trackbar_t * bar)
{
    if(bar->if_switch == true)//ʹ�ܣ�����pidֵ�����з���
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0;//δʹ�ܣ������ı�
}
