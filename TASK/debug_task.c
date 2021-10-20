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
#include "openmv.h"
#define DUBUG_MOTOR 0
int if_OsRunning(void);
void Set_OSRunningFlag(int status);

// todo  出发前做好检查
#define Blue_Route 1
#define Red_Route 0
// todo 在这里定义红蓝半场
int Os_RunningFlag = 0;

void Startdebug(void const *argument)
{
    Set_OSRunningFlag(true);
    Start_Read_Switch();
    avoid_keep();
    MV_Start();
    osDelay(100);
    MV_SendCmd(3, 0);
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
#if Blue_Route == 1
    move_by_encoder(2, 80);
    osDelay(500);
    direct_move(2, 4, 0, 1);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(1, 1);
    MV_HW_Scan(1, 1, 1);
    move_by_encoder(2, -80);
    Turn_angle(1, 180, 1);
    direct_move(2, 1, 0, 1);
    direct_move(1, 1, 1, 1);
    move_slantly(1, 120, 1500);
    Wait_Switches(3);
    Set_InitYaw(180);
    HWSwitch_Move(5, 1);
    MV_HW_Scan(1, 6, 1);
    Brick_QR_Mode(5, 1, 0, 1);
    move_slantly(3, 150, 1800);
    Turn_angle(1, -90, 0);
    direct_move(2, -1, 0, 1);
    Wait_Switches(4);
    Set_InitYaw(-90);

    // Action_Gruop(13, 1); todo对这几个函数进行封装
    // osDelay(1000);
    // Action_Gruop(8, 1);
    // osDelay(1000);
    // Action_Gruop(12, 1);
    // osDelay(1000);
    // Action_Gruop(7, 1);
    // osDelay(1000);
    move_by_encoder(2, 10);
    Turn_angle(1, 180, 0);
    Wait_Switches(1);
    Set_InitYaw(90);
    HWSwitch_Move(2, 1);
    move_by_encoder(1, -12);
    Wait_Switches(1);
    move_by_encoder(2, -78);
    Turn_angle(1, -90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
#elif RED_Route == 1
    move_by_encoder(2, 30);
    direct_move(2, 3, 0, 1);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(1, 1);
    MV_HW_Scan(1, 1, 1);
    direct_move(2, -2, 1, 1);
    move_slantly(4, 120, 1500);
    Wait_Switches(3);
    Set_InitYaw(0);
    HWSwitch_Move(5, 1);
    MV_HW_Scan(1, 6, 1);
    Brick_QR_Mode(5, 1, 0, 1);
    move_slantly(3, 150, 1800);
    Turn_angle(1, -90, 0);
    direct_move(2, -1, 0, 1);
    Wait_Switches(4);
    Set_InitYaw(-90);
    // todo在这里增加开关门的动作组运行
    move_by_encoder(2, 10);
    Turn_angle(1, 180, 0);
    Wait_Switches(1);
    Set_InitYaw(90);
    HWSwitch_Move(2, 1);
    move_by_encoder(1, -12);
    Wait_Switches(1);
    move_by_encoder(2, -78);
    Turn_angle(1, 90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
#endif
}
void Go_Home(int color)
{
    if (color != 1 && color != 2)
        return;
    move_by_encoder(2, -80);
    if (color == 1)
        Turn_angle(1, 90, 0);
    else
        Turn_angle(1, -90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
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
