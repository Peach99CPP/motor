/************************************************************************
  *      
  * FileName   : imu_pid.h   
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



#ifndef __IMU_PID_H_
#define __IMU_PID_H_
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "pid.h"
#include "math.h"
#include <stdbool.h>


float  imu_correct_val(void);

void set_imu_angle(int angle);
void set_imu_param(int p,int i,int d);
void set_imu_status(int status);
float  angle_limit(float  angle);
#endif


 

