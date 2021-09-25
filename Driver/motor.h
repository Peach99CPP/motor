#ifndef __MOTOR_H
#define __MOTOR_H
#include "tim_control.h"
#include <stdbool.h>
#include "pid.h"
#include "tim.h"
#include "uart_handle.h"
#include "delay.h"
typedef enum
{
    Channel1 = 1,
    Channel2,
    Channel3,
    Channel4
} Channel_t;
typedef struct
{
    TIM_HandleTypeDef * Tim;
    uint32_t Channel_A, Channel_B;
} PWM_CHANNEL_t;
typedef  struct
{
    GPIO_TypeDef * Port;
    uint16_t Pin;
} Enocede_IO;
typedef struct
{
    TIM_HandleTypeDef * Tim;
    HAL_TIM_ActiveChannel Active_Channel;
    uint32_t  Channel;

} IC_t;
typedef struct
{
    Enocede_IO Encoder_IO;
    IC_t IC;
    PWM_CHANNEL_t PWM ;
} motor_t;

volatile uint32_t * get_motor_channelA_ptr( int motor_id);
volatile uint32_t * get_motor_channelB_ptr( int motor_id);


void motor_init(void);
void Motor_PID_Init(void);
float read_encoder(int motor_id);
void set_motor(int motor_id, int control_val);
/****调试PID部分的函数****/
void set_motor_pid(int kp, int ki, int kd);
void set_motor_maxparam(int integrate_max, int control_output_limit);
void set_debug_motor(int status, int motor_id);
void motor_debug(void);
void set_debug_speed(int speed);
/**********************/
void show_speed(void);
void clear_motor_data(void);

extern int debug_motor_id, switch_status, debug_speed ;
extern pid_data_t motor_data[5];
extern motor_t motor1, motor2, motor3, motor4;
#endif
