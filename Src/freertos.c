/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
    #include "usmart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId debugtaskHandle;
osThreadId chassisHandle;
osThreadId track_taskHandle;
osThreadId usmart_taskHandle;
osThreadId avoid_obsHandle;
osThreadId imu_angleHandle;
osMessageQId IMU_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Startdebug(void const * argument);
void chassis_task(void const * argument);
void track_scan(void const * argument);
void usmartscan(void const * argument);
void avoid_task(void const * argument);
void IMU_decode(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of IMU_Queue */
  osMessageQDef(IMU_Queue, 20, uint8_t);
  IMU_QueueHandle = osMessageCreate(osMessageQ(IMU_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of debugtask */
  osThreadDef(debugtask, Startdebug, osPriorityNormal, 0, 128);
  debugtaskHandle = osThreadCreate(osThread(debugtask), NULL);

  /* definition and creation of chassis */
  osThreadDef(chassis, chassis_task, osPriorityNormal, 0, 512);
  chassisHandle = osThreadCreate(osThread(chassis), NULL);

  /* definition and creation of track_task */
  osThreadDef(track_task, track_scan, osPriorityHigh, 0, 128);
  track_taskHandle = osThreadCreate(osThread(track_task), NULL);

  /* definition and creation of usmart_task */
  osThreadDef(usmart_task, usmartscan, osPriorityLow, 0, 256);
  usmart_taskHandle = osThreadCreate(osThread(usmart_task), NULL);

  /* definition and creation of avoid_obs */
  osThreadDef(avoid_obs, avoid_task, osPriorityBelowNormal, 0, 128);
  avoid_obsHandle = osThreadCreate(osThread(avoid_obs), NULL);

  /* definition and creation of imu_angle */
  osThreadDef(imu_angle, IMU_decode, osPriorityAboveNormal, 0, 256);
  imu_angleHandle = osThreadCreate(osThread(imu_angle), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Startdebug */
/**
* @brief Function implementing the debug_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startdebug */
__weak void Startdebug(void const * argument)
{
  /* USER CODE BEGIN Startdebug */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Startdebug */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_track_scan */
/**
* @brief Function implementing the track_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_track_scan */
__weak void track_scan(void const * argument)
{
  /* USER CODE BEGIN track_scan */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END track_scan */
}

/* USER CODE BEGIN Header_usmartscan */
/**
* @brief Function implementing the usmart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usmartscan */
__weak void usmartscan(void const * argument)
{
  /* USER CODE BEGIN usmartscan */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usmartscan */
}

/* USER CODE BEGIN Header_avoid_task */
/**
* @brief Function implementing the avoid_obs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_avoid_task */
__weak void avoid_task(void const * argument)
{
  /* USER CODE BEGIN avoid_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END avoid_task */
}

/* USER CODE BEGIN Header_IMU_decode */
/**
* @brief Function implementing the imu_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_decode */
__weak void IMU_decode(void const * argument)
{
  /* USER CODE BEGIN IMU_decode */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_decode */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
