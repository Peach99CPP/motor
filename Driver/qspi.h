
/************************************************************************
  *      
  * FileName   : qspi.h   
  * Version    : v1.0		
  * Author     : 桃子			
  * Date       : 2021-08-24         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
*******************************************************************************/



#ifndef __QSPI_H_
#define __QSPI_H_

#include "main.h"

void qspi_init(void);
void qspi_deinit(void);
void spi_send_cmd(uint8_t cmd, uint32_t addr, uint8_t has_addr, uint8_t dummt_cycles, uint32_t count);
uint8_t qspi_receive(uint8_t* buf,uint32_t datalen);
uint8_t qspi_transmit(uint8_t* buf,uint32_t datalen);
#endif


 


