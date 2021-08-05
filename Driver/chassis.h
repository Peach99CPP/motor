#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "main.h"
#include <stdbool.h>
#include "motor.h"
#include "tim_control.h"
typedef struct chassis_structure
{
    bool _switch;//�Ƿ�ʹ�ܵ���
    double x_speed;//x��������ٶ�
    double  y_speed;//y��������ٶ�
    double  w_speed;//w������̽��ٶ�
} CHASSIS;





extern CHASSIS chassis;
void set_speed (int x,int y);
void change_switch_stage(bool status);
void chassis_synthetic_control(void);
#endif
