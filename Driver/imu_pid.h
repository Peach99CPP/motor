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

#include "ahrs.h"
#include "pid.h"
#include "math.h"
void set_imu_param(int p,int i,int d);
void set_imu_status(int status);
void turn_angle(int rt_angle);//相对角度
float  angle_limit(float  angle);
#endif


 

