#ifndef __TIM_CONTROL_H
#define __TIM_CONTROL_H
#include "main.h"
#include "math.h"
#include "time_cnt.h"
#include "motor.h"
#include "uart_handle.h"
#include "stm32f7xx_hal_tim.h"
#define ABS(x) ( (x)>0?(x):-(x) )
void All_Tim_Init(void);
extern double encoder_val[5], encoder_sum;
extern short status_flag[5];
void MY_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif

