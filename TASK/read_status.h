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
    err = -1,
    off ,
    on,
} status_;

int Get_Switch_Status(int id);
int Get_HW_Status(int id);
void Start_Read_Switch(void);
void Exit_Swicth_Read(void);
void Wait_Switches(int dir);
void Single_Switch(int switch_id);
void Set_SwitchParam(int main,int vertical);
#endif


 

