/************************************************************************
  *
  * FileName   : avoid_obs.h
  * Version    : v1.0
  * Author     : 桃子
  * Date       : 2021-08-25
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
*******************************************************************************/



#ifndef __AVOID_OBS_H_
#define __AVOID_OBS_H_

#include "main.h"

#ifndef TRIG_GPIO_Port
#define TRIG_GPIO_Port GPIOA
#define TRIG_PIN GPIO_PIN_9

#endif

#ifndef SR04_GPIO_Port
#define SR04_GPIO_Port  GPIOA
#define SR04_PIN  GPIO_PIN_6
#endif

void avoid_init(void);
uint16_t distance_convert(uint16_t raw_data);
void avoid_callback(void);
void start_avoid(void);

extern uint16_t raw_data, distance;
#endif




