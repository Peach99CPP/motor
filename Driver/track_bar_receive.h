/************************************************************************
  *      
  * FileName   : track_bar_receive.h   
  * Version    : v1.0		
  * Author     : 桃子			
  * Date       : 2021-08-21         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
*******************************************************************************/



#ifndef __TRACK_BAR_RECEIVE_H_
#define __TRACK_BAR_RECEIVE_H_
#include "main.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "string.h"
#define TRACK_UART  huart6
#define MAX_TRACK_REC_SIZE   30
#define MAX_ROW_SIZE 6
typedef enum{
    forward_bar=0,
    left_bar,
    right_bar
}track_id_t;

typedef struct
{
    uint8_t rec_data[MAX_TRACK_REC_SIZE];
    uint8_t rec_flag ;
    uint8_t current_index;
    
}Track_RX_t;

typedef struct {
    uint8_t rec_data[MAX_ROW_SIZE][MAX_TRACK_REC_SIZE];
    uint8_t current_index;
    uint8_t rec_flag;
}Track_RXRows_t;

typedef struct
{
    track_id_t id;
    uint8_t num;
    uint8_t line_flag;
    uint8_t line_num;
    pid_data_t data;
    bool if_switch;
} trackbar_t;
void Track_RX_IRQ(void);
void track_IT_handle(void);
void track_bar_init(void);
void track_decode(void);
void track_status(int id,int status);
uint8_t get_avaiable_pos(void);
void set_track_pid(int kp,int ki,int kd);
extern trackbar_t y_bar,x_leftbar,x_rightbar;
extern int dma_count;
float track_pid_cal(trackbar_t * bar);
int Get_Trcker_Num(trackbar_t *bar);
void Clear_Line(trackbar_t* bar);
#endif




