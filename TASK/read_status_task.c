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

#include "chassis.h"
#include "atk_imu.h"

osThreadId Read_Swicth_tasHandle;//������
void Read_Swicth(void const * argument);//��������



int read_task_exit = 1;//�����˳���־

short swicth_status[8];//����״̬��ֻ���ڲ����и�ֵ

#define SWITCH(x) swicth_status[(x)-1] //Ϊ��ֱ���жϿ��ر��




/**********************************************************************
  * @Name    Start_Read_Switch
  * @declaration : �����ᴥ��������
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Start_Read_Switch(void)
{
    read_task_exit = 0;//���������
    memset(swicth_status, 0, sizeof(swicth_status));//��ʼ������
    /* definition and creation of Read_Swicth_tas */
    osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityNormal, 0, 128);
    Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
}


/**********************************************************************
  * @Name    Read_Swicth
  * @declaration : ������ʵ�ֺ��ĺ���
  * @param   argument: [����/��] ������
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Read_Swicth(void const * argument)
{
    while(!read_task_exit)
    {
        //��ΪGPIO�ڱ����ó�pullup������ֻ���ڵ�ʱ�����ᴥ���ص�ͨ״̬
        if(HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
            SWITCH(1) = on;//����״̬ö��
        else
            SWITCH(1) = off;

        if(HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if(HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if(HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == GPIO_PIN_RESET)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if(HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == GPIO_PIN_RESET)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if(HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) == GPIO_PIN_RESET)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if(HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) == GPIO_PIN_RESET)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if(HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) == GPIO_PIN_RESET)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;

        osDelay(50);//�������Ƶ�ʲ���,���Կ���50ms������ˢ��
    }
    memset(swicth_status, err, sizeof(swicth_status));//��յ�δ��ʼ״̬�����ڱ�Ǵ�ʱ����δ����
    vTaskDelete(Read_Swicth_tasHandle);//�������б����Ƴ�������
    Read_Swicth_tasHandle = NULL;//����ÿ�
}



/**********************************************************************
  * @Name    Get_Switch_Status
  * @declaration : ��ȡָ��ID���ص�ͨ��״̬
  * @param   id: [����/��] �����ţ�1-8��
  * @retval   : �ÿ��ص�ͨ��״̬
  * @author  peach99CPP
***********************************************************************/
int Get_Switch_Status(int id)
{
    if(read_task_exit) return err;//ȷ����ǰ�����ڽ�����
    return SWITCH(id);//���ض�Ӧ���ص�״̬
}



/**********************************************************************
  * @Name    Exit_Swicth_Read
  * @declaration : �˳���ѯ����״̬������
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Exit_Swicth_Read(void)
{
    read_task_exit = 1;//�˱���Ϊ1 ��ʹ��������ѭ������������
}



/**********************************************************************
  * @Name    Wait_Switches
  * @declaration :��ײ�ᴥ���ص�ʵ��ȫ����
  * @param   dir: [����/��]
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Wait_Switches(int dir)
{
    /*��������ʱ�ٶȵĺ궨��,���˹��߷����ȶ�*/
#define Switch_Factor 10
#define MIN_SPEED 50

    if(read_task_exit) Start_Read_Switch();

    imu.switch_ = false;

    short flag1, flag2, x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //���ڲ����Ľ��������ݷ������ж��ٶȵķ��䷽��
        if(dir == 1) //��Y����
        {
            w1 = 2, w2 = 1;
            x_pn = 0, y_pn = 1;
        }
        else if(dir == 2 ) //��X����
        {
            w1 = 3, w2 = 4;
            x_pn = 1, y_pn = 0;
        }
        else if(dir == 3 )//��X����
        {
            w1 = 5, w2 = 6;
            x_pn = -1, y_pn = 0;

        }
        else if(dir == 4)//��Y����
        {
            w1 = 7, w2 = 8;
            x_pn = 0, y_pn = -1;
        }
    }

    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0);//����һ�������ٶȣ����ٶ��뷽������й�
    //�ȴ����ض�����
    do
    {
        flag1 = Get_Switch_Status(w1);//��ȡ״̬
        flag2 = Get_Switch_Status(w2);
        /*������һ����䣬ֻ�ڵ������ؿ���ʱ��������*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));

        if(flag1 == err || flag2 == err ) Start_Read_Switch();//��ֹ��ʱ����δ�������¿���ѭ��
        //�������
        osDelay(50);

    }
    while( flag1 == off || flag2 == off);//ֻ����������ͨ�����˳���ѭ��
    Exit_Swicth_Read();//�����˾͹ر�����
    set_speed(0, 0, 0);//����
    /*******��������Ӧ�ý�һ�����������ǣ����ǻή�ͳ��������ԣ����Բ����*******/
}
