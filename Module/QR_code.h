#ifndef __Q_R_CODE_H_
#define __Q_R_CODE_H_

#include "main.h"
#include <string.h>

typedef enum
{
    init_status =0,
    red,
    blue,
}QRcolor_t;
typedef struct 
{
    UART_HandleTypeDef * QR_uart;
    uint8_t rec_len;
    uint8_t RX_OK;
    uint8_t RX_data[20];
    QRcolor_t color;
}QR_t;
void QR_receive(void);
void QR_decode(void);
void Set_QR_Target(int color);
void Set_QR_Status(int status);
int Get_QRColor(void);
void DeInit_QRColor(void);

#endif


 

