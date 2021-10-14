/************************************************************************
  *      
  * FileName   : Wait_BackInf.h   
  * Version    : v1.0		
  * Author     : 妗冨瓙			
  * Date       : 2021-10-14         
  * Description:    
*******************************************************************************/

#ifndef __WAIT__BACK_INF_H_
#define __WAIT__BACK_INF_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdbool.h>

bool Get_TimeResult(void);             //获取计时结果
void Start_CountTime(long timelength); //开始计时
void Exit_CountTime(void);                 //强制退出计时器

#endif
