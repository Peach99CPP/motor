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
#include <stdbool.h>

#include "main.h"
typedef enum
{
  err = -1,
  off = 0,
  on = 1,
} status_;

typedef enum
{
  Primary_Head = 0,
  Low_Head = 1,
  Medium_Head
} ScanDir_t;

typedef enum
{
  PrimaryHeight = 0,
  LowestHeight,
  MediumHeight,
  HighestHeight,
} Height_t;

typedef enum
{
  toMedium = 38,
  toHighest = 39,
  toLowest = 37,
} QR_HeightVal_t;
typedef enum
{
  Not_Running = 0,
  Red_,
  Blue_,
} Game_Color_t;

int Get_Switch_Status(int id);
int Get_HW_Status(int id);
void Start_Read_Switch(void);
void Exit_Swicth_Read(void);
void Wait_Switches(int dir);
void Single_Switch(int switch_id);
void HWSwitch_Move(int dir, int enable_imu); //使用红外开关进行移动
void Set_SwitchParam(int main, int vertical);
void MV_HW_Scan(int color, int dir, int enable_imu);
void Brick_QR_Mode(int dir, int color, int QR, int imu_enable);
void Start_HeightUpdate(void);
void QR_Scan(int status, int color, int dir, int enable_imu);
void Kiss_Ass(int dir, int enable_imu);
int Get_Height_Switch(int id);
int Get_MV_Servo_Flag(void);
int Get_Height(void);
//只在内部使用的函数
void Inf_Servo_Height(int now_height);
void QR_Mode_Height(void) ;

#endif
