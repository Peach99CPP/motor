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

void avoid_init(void);
uint16_t distance_convert(uint16_t raw_data);
void avoid_callback(void);
void start_avoid(void);
void avoid_keep(void);
void exit_avoid(void);
extern uint16_t raw_data, distance;
#endif




