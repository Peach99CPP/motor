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
    bool enable_switch;//�Ƿ�ʹ�ܵ���
    double x_speed;//x��������ٶ�
    double  y_speed;//y��������ٶ�
    double  w_speed;//w������̽��ٶ�
} CHASSIS_t;





extern CHASSIS_t chassis;


void w_speed_set(float w_speed);
void speed_variation(float x_var, float y_var, float w_var);
void set_speed (int x, int y, int w);
void set_chassis_status(bool status);
void chassis_synthetic_control(void);


int Get_X_speed(void);
int Get_Y_speed(void);
int Get_W_speed(void);

void Set_Dir_Speed(char dir, int speed);

void Wait_OKInf(int type,long wait_time);

float get_chassis_speed(char dir);

bool inte_move(int type, int dir, int val, int edge, int imu_if, long wait_time);//�˶����ۺ�

extern float motor_target[5];
extern float control_val[5];
#endif
