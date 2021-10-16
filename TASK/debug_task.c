#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
#include "avoid_obs.h"
#include "atk_imu.h"
#include "track_bar_receive.h"
#include "servo.h"
#define DUBUG_MOTOR 0
int if_OsRunning(void);
void Set_OSRunningFlag(int status);

int Os_RunningFlag = 0;

void Startdebug(void const *argument)
{
    Set_OSRunningFlag(true);
    Start_Read_Switch();
    avoid_keep();
#if DUBUG_MOTOR == 1
    track_status(1, 0);
    track_status(2, 0);
    Set_IMUStatus(false);
#endif

    while (1)
    {
#if DUBUG_MOTOR == 1
        motor_debug();
#endif
        osDelay(1);
    }
}

void Global_Debug(void)
{
   move_by_encoder(2, 60);
    direct_move(2, 3, 0, 1);
    track_status(1, 0);
    track_status(2, 0);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(1, 1);
    MV_HW_Scan(1, 1, 1);
    turn_angle(1, 180);
    direct_move(2, 1, 0, 1);
    direct_move(1, 1, 1, 1);
    direct_move(2, 1, 1, 1);
    move_slantly(1, 120, 1500);
    Wait_Switches(3);
    Set_InitYaw(0);
    HWSwitch_Move(5, 1);
    move_slantly(3, 120, 1500);
    turn_angle(1, -90);
    osDelay(1000);
    direct_move(2, -1, 0, 1);
    Wait_Switches(4);
    Set_InitYaw(180);
    Action_Gruop(13, 1);
    osDelay(1000);
    Action_Gruop(8, 1);
    osDelay(1000);
    Action_Gruop(12, 1);
    osDelay(1000);
    Action_Gruop(7, 1);
    osDelay(1000);
    move_by_encoder(2, 10);
    turn_angle(1, 180);
    osDelay(1000);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(2, 1);
    move_by_encoder(1, -55);
    track_status(1, 0);
    track_status(2, 0);
    Wait_Switches(1);
}
int if_OsRunning(void)
{
    return Os_RunningFlag;
}
void Set_OSRunningFlag(int status)
{
    Os_RunningFlag = status;
}

void Goto_Warehouse(void)
{
    ;
}
