#ifndef __MOTOR_H
#define __MOTOR_H
#include "tim_control.h"
#include <stdbool.h>
#include "pid.h"
void Motor_PID_Init();
short read_encoder(int motor_id);
void set_motor(int motor_id, short control_val);
void set_motor_pid(int kp, int ki, int kd);
void set_motor_maxparam(int integrate_max, int control_output_limit);
void pid_param_init(pid_paramer_t *controler, float params[]); //此函数后期移动回去pid.c中
extern pid_data_t motor_controler[5];
#endif
