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
void Startdebug(void const *argument)
{
    Start_Read_Switch();
    avoid_keep();
//    track_status(1, 0);
//    track_status(2, 0);
//    Set_IMUStatus(false);

    while (1)
    {
//        motor_debug();
        osDelay(1);
    }
}
