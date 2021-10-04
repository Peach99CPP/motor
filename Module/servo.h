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

#define MAX_SERVO_SIZE 100

typedef struct{
    UART_HandleTypeDef * uart;
    uint8_t current_index;
    uint8_t cmd_buffer[MAX_SERVO_SIZE];
}ServoControler_t;

typedef enum{
    Lowest =1,
    Medium,
    Highest,
    Bar
}Servo_ID_t;
void Cmd_Convert(int cmd);

void Servo_Uart_Send(void);
void Single_Control(int id, int control_mode, int angle, int  time, int delay);
void Action_Gruop(int  id,int  times);

extern ServoControler_t servo_controler;

#endif


 

