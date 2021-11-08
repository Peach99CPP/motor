#ifndef __Q_R_CODE_H_
#define __Q_R_CODE_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>
typedef enum
{
    init_status =0,
    red,
    blue,
}QRcolor_t;
typedef struct 
{
    UART_HandleTypeDef * QR_uart;
    bool enable_switch;
    uint8_t rec_len;
    uint8_t RX_OK;
    uint8_t RX_data[20];
    QRcolor_t color;
}QR_t;

void QR_Mode_Init(bool status, QRcolor_t target_color);
void QR_receive(void);
void QR_decode(void);

void Set_QR_Target(QRcolor_t color);
void Set_QR_Status(bool status);

int Get_QRColor(void);
bool Return_QRMode(void);
void DeInit_QRColor(void);

#endif


 

