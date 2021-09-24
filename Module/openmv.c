#include "openmv.h"
#include "servo.h"
#include "chassis.h"
int mv_param;
mvrec_t mv_rec;
mv_t MV =
    {
        .mv_uart = &huart4,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //��ʼ������

/**********************************************************************
  * @Name    cmd_encode
  * @declaration : ����Э����뷢�͵�����
  * @param   event_id: [����/��]  ʱ�������
**			 param: [����/��] ����
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void cmd_encode(const uint8_t event_id, int param)
{
    static uint8_t pos_flag;
    if (param > 0)
        pos_flag = 1;
    else
    {
        pos_flag = 2;
        param *= -1;
    }

    uint8_t h_byte, l_byte; //��ȡ�����ĸ�8λ�͵�8λ
    h_byte = (param >> 8);
    l_byte = (param & 0xff);
    //����ͨѶЭ��
    MV.mv_cmd[0] = START_BYTE; //֡ͷ
    MV.mv_cmd[1] = event_id;   //�������¼�id
    MV.mv_cmd[2] = pos_flag;
    MV.mv_cmd[3] = l_byte;                                           //������8λ
    MV.mv_cmd[4] = h_byte;                                           //������8λ
    MV.mv_cmd[5] = (uint8_t)(event_id + pos_flag + h_byte + l_byte); //��У��
    MV.mv_cmd[6] = END_BYTE;                                         //֡β
}
void MV_SendCmd(const uint8_t event_id, const int param)
{
    cmd_encode(event_id, param);                                 //���ݻ�õĲ�������cmd����
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //��cmd���ͳ�ȥ
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //��cmd�������³�ʼ��
}

/**********************************************************************
  * @Name    MV_IRQ
  * @declaration :  openmvͨѶ���жϴ�����
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void MV_IRQ(void)
{
    if (MV.RX_Status < 2) //δ�������
    {
        uint8_t rec_data = MV.mv_uart->Instance->RDR; //��ȡ���ν��յ�ֵ
        if (rec_data == START_BYTE && MV.RX_Status == 0)
        {
            MV.RX_Status = 1; //����֡ͷ������ǣ�ֱ���˳�
            MV.rec_len = 0;
            return;
        }
        if (MV.RX_Status == 1) //�յ�֡ͷ��
        {
            if (rec_data == END_BYTE && MV.rec_len == 6) //֡ͷ֡β�����յ���,Ϊ�˱���PID��������г�����Ҫָ�����ճ���
            {
                MV.RX_Status = 2; //��ǽ������
                MV_rec_decode();  //�Խ��յ������ݽ��н������
            }
            else
            {
                //δ�յ�֡β����ʱ����������
                MV.rec_buffer[MV.rec_len++] = rec_data; //��������
                if (MV.rec_len == MAX_REC_SIZE)
                    MV.RX_Status = 0; //��ֹ��Ϊ�����¿���
            }
        }
    }
}

/**********************************************************************
  * @Name    MV_rec_decode
  * @declaration : �жϽ�����ɺ󣬶Խ��յ����ݽ��н���
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void MV_rec_decode(void)
{
    static int pn = 1; //������־��
    if (MV.rec_buffer[0] + MV.rec_buffer[1] + MV.rec_buffer[2] + MV.rec_buffer[3] == MV.rec_buffer[4])
    {
        //���ݲ������ݶԲ������д���
        if (MV.rec_buffer[1] == 1)
            pn = 1;
        else
            pn = -1;
        mv_rec.event = MV.rec_buffer[0];
        mv_rec.param = (MV.rec_buffer[2] + (MV.rec_buffer[3] << 8)) * pn;
    }
    MV.rec_len = 0;
    MV.RX_Status = 0;
    //������֮��ǵ����³�ʼ���ṹ���е�rec_len��RX_status�������������
    ;
}

/****�����ǵײ�ʵ�֣��������ϲ��Ӧ��****/

/**********************************************************************
  * @Name    MV_Ball
  * @declaration :����MVҪ���ʲô��ɫ����
  * @param   color: [����/��]  Ŀ�������ɫ
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void MV_Ball(int color)
{
    if (color == red)
        MV_SendCmd(1, 1);
    else if (color == blue)
        MV_SendCmd(1, 2);
}

/**********************************************************************
  * @Name    MV_PID
  * @declaration :��MV����PID�ź�
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_PID(void)
{
    MV_SendCmd(2, 0);
}

/**********************************************************************
  * @Name    MV_SendOK
  * @declaration :��openmv����OK�ź�
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void MV_SendOK(void)
{
    MV_SendCmd(3, 0);
}

void MV_Decode(void)
{
#define Catch_ 1
#define MVPID_THRESHOLD 10
    if (mv_rec.event == 1)
    {
        if (mv_rec.param == 1)
            Action_Gruop(Catch_, 1);
        else if (mv_rec.param == 2)
            Action_Gruop(Catch_, 1);
    }
    else if (mv_rec.event == 2)
    {
        if (ABS(mv_rec.param) < MVPID_THRESHOLD)
        {
            set_speed(0, 0, 0);
            MV_SendOK();
        }
        else
            set_speed(mv_rec.param, 0, 0);
    }
}
