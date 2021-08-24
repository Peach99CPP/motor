/************************************************************************
  *
  * FileName   : uart_handle.h
  * Version    : v1.0
  * Author     : 桃子
  * Date       : 2021-08-08
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
*******************************************************************************/



#ifndef __UART_HANDLE_H_
#define __UART_HANDLE_H_
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "string.h"
#include <stdio.h>

void U1_IRQHandler(void);
void uart1_decode(void);
void printf_init(void);
#endif




