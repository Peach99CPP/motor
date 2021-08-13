/************************************************************************
  *
  * FileName   : delay.h
  * Version    : v1.0
  * Author     : 桃子
  * Date       : 2021-08-10
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
*******************************************************************************/



#ifndef __DELAY_H_
#define __DELAY_H_
#include "tim.h"
typedef unsigned           int uint32_t;
typedef enum
{
    up_count = 1,
    dowm_count
} count_mode;
typedef struct
{

    count_mode mode;
    TIM_HandleTypeDef * cnt_tim;
    uint32_t period_val;
} time_t;
extern time_t sys_time;
void delay_init(TIM_HandleTypeDef * ptr, count_mode mode);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
#endif




