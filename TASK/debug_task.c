#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
#include "avoid_obs.h"
#include "atk_imu.h"
#include "track_bar_receive.h"
#define DUBUG_MOTOR 0


void Startdebug(void const *argument)
{
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
    move_slantly(2, 150, 2500);
    Comfirm_Online(1);
    direct_move(2, 3, 1,1);
    while (!get_count_line_status())
        osDelay(10);
    osDelay(500);
    move_by_encoder(1, 20);
    while (!get_enocdermove_status())
        osDelay(10);
    osDelay(500);
    move_slantly(1, 150, 1150);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(1);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(2);
    move_by_encoder(2, -200);
    while (!get_enocdermove_status())
        osDelay(10);
    turn_angle(1, 180);
    while (!get_turn_status())
        osDelay(10);
    osDelay(300);
    direct_move(2, 3, 0,1);
    while (!get_count_line_status())
        osDelay(10);
    osDelay(500);
    turn_angle(1, 180);
}
