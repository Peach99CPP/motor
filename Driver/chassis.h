#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "main.h"
#include <stdbool.h>
typedef struct chassis_structure{
    bool _switch;
    float x_speed;
    float y_speed;
}chassis;
void set_speed (int x,int y);
void change_switch_stage(bool stage);
#endif
