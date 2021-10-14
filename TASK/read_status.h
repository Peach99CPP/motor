/************************************************************************
  *      
  * FileName   : read_status.h   
  * Version    : v1.0		
  * Author     : 妗冨瓙			
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
  off = 0,
  on = 1,
} status_;

typedef enum
{
  Primary_Head=0,
  Low_Head =1,
  Medium_Head
}ScanDir_t;

typedef enum
{
  PrimaryHeight= 0,
  LowestHeight ,
  MediumHeight,
  HighestHeight,
}Height_t;

typedef enum
{
  Not_Running=0,
  Red_ ,
  Blue_,
}Game_Color_t;



int Get_Switch_Status(int id);
int Get_HW_Status(int id);
void Start_Read_Switch(void);
void Exit_Swicth_Read(void);
void Wait_Switches(int dir);
void Single_Switch(int switch_id);
void HWSwitch_Move(int dir, int enable_imu); //使用红外开关进行移动
void Set_SwitchParam(int main, int vertical);
void MV_HW_Scan(int color, int dir, int enable_imu);
void Brick_Mode(int dir, int color,int enable_imu);
void Start_HeightUpdate(void);
void QR_Scan(int status, int color, int dir, int enable_imu);
void Kiss_Ass(int dir,int enable_imu);
int Get_Height_Switch(int id);
int Get_MV_Servo_Flag(void);
int Get_Height(void);
#endif
