#ifndef __CHASSIS_H_
#define __CHASSIS_H_


#endif




#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "main.h"
#include <stdbool.h>
#include "motor.h"
#include "tim_control.h"
typedef struct chassis_structure
{
    bool enable_switch;//是否使能底盘
    double x_speed;//x方向底盘速度
    double  y_speed;//y方向底盘速度
    double  w_speed;//w方向底盘角速度
} CHASSIS_t;





extern CHASSIS_t chassis;


void w_speed_set(float w_speed);
void speed_variation(float x_var, float y_var, float w_var);
void set_speed (int x, int y, int w);
void set_chassis_status(bool status);
void chassis_synthetic_control(void);

void Wait_OKInf(int type,long wait_time);

float get_chassis_speed(char dir);


extern float motor_target[5];
extern float control_val[5];
#endif
