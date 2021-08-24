/************************************************************************
  *      
  * FileName   : iic.h   
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



#ifndef __IIC_H_
#define __IIC_H_

#include "i2c.h"
#include "FreeRTOS.h"
#include "queue.h"

#define I2C_EVENT_SUCCESSFUL 1 << 0
#define I2C_EVENT_ERROR 1 << 1

extern  QueueHandle_t i2c_queue;

void HAL_I2C1_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C1_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C1_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C1_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C1_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

void i2c1_init(void);
int i2c1_transmit(uint8_t slave_address, uint8_t is_write, uint8_t *pdata, uint8_t count);
int i2c1_single_write(uint8_t slave_address,uint8_t reg_address,uint8_t reg_data);
int i2c1_single_read(uint8_t slave_address,uint8_t reg_address,uint8_t *reg_data);
int i2c1_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);
int i2c1_multi_write(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);

#endif


 

