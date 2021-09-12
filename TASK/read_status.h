/************************************************************************
  *      
  * FileName   : read_status.h   
  * Version    : v1.0		
  * Author     : 桃子			
  * Date       : 2021-09-12         
  * Description:    
*******************************************************************************/

#ifndef __READ_STATUS_H_
#define __READ_STATUS_H_
    
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#include "main.h"
typedef enum
{
    off = 0,
    on
} switch_status;

int Get_Switch_Status(int id);
void Start_Read_Switch(void);
void Exit_Swicth_Redad(void);
#endif


 

