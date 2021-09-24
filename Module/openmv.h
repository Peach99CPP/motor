#ifndef __OPENMV_H_
#define __OPENMV_H_

#include "string.h"

#include "main.h"
#include "usart.h"

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
    red=1,
    blue
}mvcolor_t;
typedef struct
{
    int event;
    int param;
}mvrec_t;


void cmd_encode(const uint8_t event_id,int  param);
void MV_SendCmd(const uint8_t event_id,const int  param);
void MV_IRQ(void);
void MV_rec_decode(void);

void MV_Ball(int color);
void MV_Decode(void);
void MV_SendOK(void);
void MV_PID(void);

#endif



 

