/* ************************************************************
  *
  * FileName   : read_status.c
  * Version    : v1.0
  * Author     : peach99CPP
  * Date       : 2021-09-12
  * Description:
  ******************************************************************************
 */
#include "read_status.h "

osThreadId Read_Swicth_tasHandle;
void Read_Swicth(void const * argument);



int read_task_exit = 0;

short swicth_status[8];
#define SWITCH(x) swicth_status[(x)-1] //��˿ɶ��Ժ��ڴ濪֧


void Start_Read_Switch(void)
{
    read_task_exit = 0;//���������
    memset(swicth_status, 0, sizeof(swicth_status));//��ʼ������
    /* definition and creation of Read_Swicth_tas */
    osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityNormal, 0, 128);
    Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
}

void Read_Swicth(void const * argument)
{
    while(1)
    {
        if(HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_SET)
            SWITCH(1) = on;
        else
            SWITCH(1) = off;

        if(HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_SET)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if(HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_SET)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if(HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == GPIO_PIN_SET)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if(HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == GPIO_PIN_SET)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if(HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) == GPIO_PIN_SET)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if(HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) == GPIO_PIN_SET)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if(HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) == GPIO_PIN_SET)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;

        osDelay(50);//�������Ƶ�ʲ���,���Կ���50ms������ˢ��
    }
    memset(swicth_status, -1, sizeof(swicth_status));//��յ�δ��ʼ״̬�����ڱ�Ǵ�ʱ����δ����
    vTaskDelete(Read_Swicth_tasHandle);
    Read_Swicth_tasHandle = NULL;
}

int Get_Switch_Status(int id)
{
    return SWITCH(id);
}
void Exit_Swicth_Redad(void)
{
    read_task_exit = 1;
    vTaskDelete(Read_Swicth_tasHandle);
    Read_Swicth_tasHandle = NULL;
}