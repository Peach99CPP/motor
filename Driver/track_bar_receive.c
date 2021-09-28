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

uint8_t track_dma[MAX_LINE][BUFF_SIZE] = {0}, dma_trans_pos = 0; //DMA���յ�����

trackbar_t y_bar, x_leftbar, x_rightbar; //����Ѱ����
//��ʼ��PID����
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
  * @declaration : ����Ѱ�����PID����API
  * @param   kp: [����/��] �Ŵ�10����p
**			 ki: [����/��] �Ŵ�10����i
**			 kd: [����/��] �Ŵ�10����d
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
  * @declaration : ��Ѱ������Ҫ����ر������г�ʼ��������
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void track_bar_init(void) //��صĳ�ʼ������
{
    dma_count = 0;
    dma_trans_pos = 0;
    //���������ĳ�ʼ��
    y_bar.id = forward_bar; //��ע���
    y_bar.line_num = 0;
    y_bar.num = 0;
    y_bar.line_num = 0;
    y_bar.data.expect = 0;  //ѭ����Ŀ��ֵ��Ϊ0
    y_bar.if_switch = true; //ʹ��

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
  * @declaration : ��ȡ���õ������±�
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
uint8_t get_avaiable_pos(void)
{
    //�°汾
    for (uint8_t i = 0; i < MAX_LINE; ++i) //��������index
    {
        if (track_dma[i][0] == START_BYTE && track_dma[i][BUFF_SIZE-1] == END_BYTE) //����Э�����
        {
            return i;
        }
    }
    return 0XFF; //һ�����Ҳ�������������ֵ���ڵ���ʱ�жϷ���ֵ�Ǵ���ֵ���Լ��ٳ���Ŀ���
}

/**********************************************************************
  * @Name    track_decode
  * @declaration :��DMA�յ������ݽ��н�����㣬ʵ��ѭ������ܵĺ��Ĵ���
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void track_decode(void)
{
    /***��غ궨��****/
#define EDGE_THRESHOLD 4 //�ڱ�Ե����ģʽ�£����ŵ�����ʱΪ��Ч����
#define NUM_THRESHOLD 6  //�Ǳ�Ե�߼����£��жϵ����ߵ�����
#define MIN_NUM 2        //��ѹ��֮�󣬹����˲Ż����һ���ߣ����ݵƵ��������м���
#define EDGE_VAL 7       //��Ե����״̬�µ�ѭ����������ֵ

    times_counts++;                   //�ܵĴ���������鿴�� ���ݿ����ж��Ƿ�DMA
    dma_count--;                      //��������-1
    uint8_t pos = get_avaiable_pos(); //��ȡ����index
    uint8_t led_num = 0;
    float track_value = 0, temp_val; //��صı�������

    //����ĺͼ��鿴�������
    if ((uint8_t)(track_dma[pos][2] + track_dma[pos][4]+track_dma[pos][6]) == track_dma[pos][BUFF_SIZE-2]) //��У��
    {
        for (uint8_t bar_id = 1; bar_id <= BUFF_SIZE - 4; bar_id += 2)
        {
            track_value = 0;
            temp_val= 0;
            led_num = 0;
            for (uint8_t i = 0; i < 8; ++i)
            {
                temp_val = (bool)(((track_dma[pos][bar_id+1] << i) & 0x80)) * track_weight[i]; //���ݵ��������Ȩ�صõ�����ֵ
                if (temp_val != 0)
                    led_num++; //�������ĵ�����
                track_value += temp_val;
            }
            switch (track_dma[pos][bar_id]) //�ж�Ѱ����ID
            {
            case 1:
                y_bar.data.feedback = track_value; //��ֵ
                y_bar.num = led_num;               //�õ��Ƶ�����
                if (y_bar.num >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
                {
                    /*�����������
                *һ�ǵ����ڷǱ�Եʱ����ʱ�Ƶ������Ƚ϶�
                *�����ڱ�Ե��ʱ����ʱ��������������Ҫ�Ӷ����ж�
                */
                    y_bar.line_flag = 1; //��ʱ������
                }
                if (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL) //��Ե���ߵ�����£����⴦��
                    y_bar.data.feedback = 0;                                                               //Ҫ���ڶ��ߵ��ж�֮����0��Ϊ�˷�ֹ��ʱ����ƫ��
                if (y_bar.line_flag && y_bar.num <= MIN_NUM)
                {
                    //ʹ�ô˻���Ϊ�˱�����ͣ�������϶������ߵ�����һֱ�ظ�����
                    y_bar.line_flag = 0; //�������
                    y_bar.line_num++;    //����Ŀ��һ
                }
                break;
            case 2:
                x_leftbar.data.feedback = track_value;
                x_leftbar.num = led_num;

                if (x_leftbar.num >= NUM_THRESHOLD || (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL))
                {
                    x_leftbar.line_flag = 1; //��ǵ�������
                }
                if (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL)
                    x_leftbar.data.feedback = 0;
                if (x_leftbar.line_flag && x_leftbar.num <= MIN_NUM) //������Ϊ������ͣ�������µ��ظ���������
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
            default: //ɶҲ����
                ;
            }
        }
        memset(track_dma[pos], 0, sizeof(track_dma[pos])); //���õ�����0���ȴ��´α����
    }
}

/**********************************************************************
  * @Name    track_IT_handle
  * @declaration : ��������ж�
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void track_IT_handle(void)
{
    uint8_t pos = get_avaiable_pos(); //ͨ����ȡ�������Ƿ��б�����ȥ
    if (pos != 0xff)
    {
        dma_count++;
        dma_trans_pos = ((dma_trans_pos + 1) >= MAX_LINE) ? 0 : dma_trans_pos + 1; //�˽�����������ɣ����������н��������н���
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t *)track_dma[dma_trans_pos], BUFF_SIZE);
    }
    //�������ԭ��������������
    else
    {
        memset(track_dma, 0, sizeof(track_dma)); //��յ������ò���ϰ��Ϊֹ
        HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t *)track_dma[dma_trans_pos], BUFF_SIZE);
    }
}

/**********************************************************************
  * @Name    HAL_UART_RxCpltCallback
  * @declaration : �����޸ĵĽ�������жϺ���
  * @param   huart: [����/��]  ������Ӧ�Ĵ���
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &TRACK_UART) //Ѱ�����Ӧ�Ĵ��ڣ�Ϊ����ֲ�ԣ��������ýṹ��
    {
        track_IT_handle();
    }
}

/**********************************************************************
  * @Name    track_pid_cal
  * @declaration : Ѱ����pid���㺯��
  * @param   bar: [����/��] ��һ��Ѱ����
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
float track_pid_cal(trackbar_t *bar)
{
    if (bar->if_switch == true) //ʹ�ܣ�����pidֵ�����з���
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0; //δʹ�ܣ������ı�
}

/**********************************************************************
  * @Name    track_status
  * @declaration :����Ѱ����״̬
  * @param   id: [����/��]  ����1Ϊ��ֱ ��2Ϊˮƽ
**			 status: [����/��]  ������ر�
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void track_status(int id, int status)
{
    if (id == 1) //y����
        y_bar.if_switch = status;
    else if (id == 2) //x����
    {
        x_leftbar.if_switch = status;
        x_rightbar.if_switch = status;
    }
}
