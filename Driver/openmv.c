#include "openmv.h"
mv_t MV =
{
    .mv_uart = &huart3,
    .mv_cmd = {0},
    .rec_buffer = {0},
    .rec_len = 0,
    .RX_Status = 0
};//��ʼ������

void cmd_encode(const uint8_t event_id, const uint16_t  param)
{
    uint8_t h_byte, l_byte;//��ȡ�����ĸ�8λ�͵�8λ
    h_byte = (param >> 8);
    l_byte = (param & 0xff);
    //����ͨѶЭ��
    MV.mv_cmd[0] = START_BYTE;//֡ͷ
    MV.mv_cmd[1] = event_id;//�������¼�id
    MV.mv_cmd[2] = h_byte;//������8λ
    MV.mv_cmd[3] = l_byte;//������8λ
    MV.mv_cmd[4] = (uint8_t)(event_id + h_byte + l_byte);//��У��
    MV.mv_cmd[5] = END_BYTE;//֡β
}
void MV_SendCmd(const uint8_t event_id, const uint16_t  param)
{
    cmd_encode(event_id, param);//���ݻ�õĲ�������cmd����
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff);//��cmd���ͳ�ȥ
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));//��cmd�������³�ʼ��
}
void MV_IRQ(void)
{
    if(MV.RX_Status < 2) //δ�������
    {
        uint8_t rec_data = MV.mv_uart->Instance->RDR; //��ȡ���ν��յ�ֵ
        if(rec_data == START_BYTE)
        {
            MV.RX_Status = 1;    //����֡ͷ������ǣ�ֱ���˳�
            return;
        }
        if( MV.RX_Status == 1 ) //�յ�֡ͷ��
        {
            if(rec_data == END_BYTE)//֡ͷ֡β�����յ���
            {
                MV.RX_Status = 2; //��ǽ������
                MV_rec_decode();//�Խ��յ������ݽ��н������
            }
            else
            {
                //δ�յ�֡β����ʱ����������
                MV.rec_buffer[MV.rec_len++] = rec_data;//��������
                if(MV.rec_len == MAX_REC_SIZE) MV.RX_Status = 0; //��ֹ��Ϊ�����¿���
            }

        }


    }

}
void MV_rec_decode(void)
{
    //������֮��ǵ����³�ʼ���ṹ���е�rec_len��RX_status�������������
    ;
}