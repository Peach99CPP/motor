#ifndef __OPENMV_H_
#define __OPENMV_H_

#include "string.h"

#include "main.h"
#include "usart.h"
#include "uart_handle.h"

#define START_BYTE 0XFF
#define END_BYTE  0X99
#define BUFFER_SIZE 7
#define MAX_REC_SIZE 20

typedef struct
{
    UART_HandleTypeDef* mv_uart;
    uint8_t mv_cmd[BUFFER_SIZE];
    uint8_t rec_buffer[MAX_REC_SIZE];
    uint8_t rec_len;
    uint8_t RX_Status;
}mv_t;
typedef enum
{
    red_color=1,
    blue_color
}mvcolor_t;
typedef struct
{
    int event;
    int param;
}mvrec_t;

typedef enum
{
    ladder_type=0,
    bar_type
}mv_type_t;

void cmd_encode(const uint8_t event_id,int  param);
void MV_SendCmd(const uint8_t event_id,const int  param);
void MV_IRQ(void);
void MV_rec_decode(void);

int Get_Stop_Signal(void);
void Disable_StopSignal(void);
void Enable_StopSignal(void);

void OpenMV_ChangeRoi(int roi);

void MV_Decode(void);
void MV_SendOK(void);
void MV_PID(void);

void MV_Scan_Bar(mvcolor_t color);//扫描条形平台
//扫描阶梯平台
void MV_Scan_Low(mvcolor_t color);
void MV_Scan_High(mvcolor_t color);
//开始与结束
void MV_Start(void);
void MV_Stop(void);
#endif



 

