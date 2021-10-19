/************************************************************************
  *
  * FileName   : servo.h
  * Version    : v1.0
  * Author     : 桃子
  * Date       : 2021-09-10
  * Description:
  * Function List:
    1. ....
       <version>:
  <modify staff>:
          <data>:
   <description>:
    2. ...
*******************************************************************************/

#ifndef __SERVO_H_
#define __SERVO_H_

#include <string.h>

#include "main.h"
#include "usart.h"
#include <stdbool.h>

#define MAX_SERVO_SIZE 100
#define MAX_SERVO_REC_SIZE 20
typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t current_index;
    uint8_t cmd_buffer[MAX_SERVO_SIZE];
    uint8_t rec_buffer[MAX_SERVO_REC_SIZE];
    uint8_t rec_index;
} ServoControler_t;

typedef enum
{
    Lowest = 1,
    Medium,
    Highest,
    Bar,
} Servo_ID_t;
void Cmd_Convert(int cmd); //ת��ָ��

void Servo_Uart_Send(void);                                                    //ͨ�����ڷ���
void Single_Control(int id, int control_mode, int angle, int time, int delay); //���Ƶ������
void Action_Gruop(int id, int times);                                          //���ж�����
int Get_Servo_Flag(void);                                                      //��ȡ������״̬
void Servo_RX_IRQ(void);                                                       //��ؽ����жϴ���
void Enable_ServoFlag(void);                                                   //ʹ��״̬λ
void Disable_ServoFlag(void);                                                  //���״̬λ

void Wait_Servo_Signal(long wait_time_num); //������ʱ���� �ȴ����ָ��
void Ass_Door(int status);                  //��ƨ�ɵ��Ž��򵹽�ȥ
void Lateral_infrared(int status);          //����������ĺ���
void Baffle_Control(int up_dowm);           //���Ƶ��������
void Different_Dir(int if_left);            //����ֿ߲⻹���ұֿ߲�

bool Get_IFUP(void);        //��е���Ƿ��Ѿ�����
void Set_IFUP(bool status); //���û�е��״̬

extern ServoControler_t servo_controler;
extern uint8_t mv_rec_flag;
#endif
