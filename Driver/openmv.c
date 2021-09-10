#include "openmv.h"
mv_t MV =
{
    .mv_uart = &huart3,
    .mv_cmd = {0},
    .rec_buffer = {0},
    .rec_len = 0,
    .RX_Status = 0
};//初始化变量

void cmd_encode(const uint8_t event_id, const uint16_t  param)
{
    uint8_t h_byte, l_byte;//获取参数的高8位和低8位
    h_byte = (param >> 8);
    l_byte = (param & 0xff);
    //定义通讯协议
    MV.mv_cmd[0] = START_BYTE;//帧头
    MV.mv_cmd[1] = event_id;//触发的事件id
    MV.mv_cmd[2] = h_byte;//参数高8位
    MV.mv_cmd[3] = l_byte;//参数低8位
    MV.mv_cmd[4] = (uint8_t)(event_id + h_byte + l_byte);//和校验
    MV.mv_cmd[5] = END_BYTE;//帧尾
}
void MV_SendCmd(const uint8_t event_id, const uint16_t  param)
{
    cmd_encode(event_id, param);//根据获得的参数编码cmd数组
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff);//将cmd发送出去
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));//将cmd数组重新初始化
}
void MV_IRQ(void)
{
    if(MV.RX_Status < 2) //未接收完成
    {
        uint8_t rec_data = MV.mv_uart->Instance->RDR; //读取本次接收的值
        if(rec_data == START_BYTE)
        {
            MV.RX_Status = 1;    //读到帧头，做标记，直接退出
            return;
        }
        if( MV.RX_Status == 1 ) //收到帧头后
        {
            if(rec_data == END_BYTE)//帧头帧尾都接收到了
            {
                MV.RX_Status = 2; //标记接收完成
                MV_rec_decode();//对接收到的内容进行解码操作
            }
            else
            {
                //未收到帧尾，此时是数据内容
                MV.rec_buffer[MV.rec_len++] = rec_data;//存入数组
                if(MV.rec_len == MAX_REC_SIZE) MV.RX_Status = 0; //防止因为出错导致卡死
            }

        }


    }

}
void MV_rec_decode(void)
{
    //处理完之后记得重新初始化结构体中的rec_len和RX_status变量，避免出错
    ;
}