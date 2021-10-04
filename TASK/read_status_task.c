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
#include "openmv.h"

#define Height_HW 5

osThreadId Read_Swicth_tasHandle;       //������
void Read_Swicth(void const *argument); //��������

int read_task_exit = 1; //�����˳���־
static short height_status = 0;
short swicth_status[8]; //����״̬��ֻ���ڲ����и�ֵ
short HW_Switch[4];     //���⿪�ص�״̬
int MIN_ = 60;
int VERTICAL = 10;

#define SWITCH(x) swicth_status[(x)-1] //Ϊ��ֱ���жϿ��ر��
#define HW_SWITCH(X) HW_Switch[(X)-1]
/**********************************************************************
  * @Name    Start_Read_Switch
  * @declaration : �����ᴥ��������
  * @param   None
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Start_Read_Switch(void)
{
    if (read_task_exit)
    {
        read_task_exit = 0; //���������
        //��ʼ������
        memset(swicth_status, 0, sizeof(swicth_status));
        memset(HW_Switch, 0, sizeof(HW_Switch));

        /* definition and creation of Read_Swicth_tas */
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityNormal, 0, 128);
        Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
    }
}

/**********************************************************************
  * @Name    Read_Swicth
  * @declaration : ������ʵ�ֺ��ĺ���
  * @param   argument: [����/��] ������
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Read_Swicth(void const *argument)
{
    while (!read_task_exit)
    {
        //��ΪGPIO�ڱ����ó�pullup������ֻ���ڵ�ʱ�����ᴥ���ص�ͨ״̬
        if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
            SWITCH(1) = on; //����״̬ö��
        else
            SWITCH(1) = off;

        if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if (HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == GPIO_PIN_RESET)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == GPIO_PIN_RESET)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if (HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) == GPIO_PIN_RESET)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if (HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) == GPIO_PIN_RESET)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if (HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) == GPIO_PIN_RESET)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;

        if (HAL_GPIO_ReadPin(HW_S1_GPIO_Port, HW_S1_Pin) == GPIO_PIN_SET)
            HW_SWITCH(1) = off;
        else
            HW_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_S2_GPIO_Port, HW_S2_Pin) == GPIO_PIN_SET)
            HW_SWITCH(2) = off;
        else
            HW_SWITCH(2) = on;

        if (HAL_GPIO_ReadPin(HW_S3_GPIO_Port, HW_S3_Pin) == GPIO_PIN_SET)
            HW_SWITCH(3) = off;
        else
            HW_SWITCH(3) = on;
        if (HAL_GPIO_ReadPin(HW_S4_GPIO_Port, HW_S4_Pin) == GPIO_PIN_SET)
            HW_SWITCH(4) = off;
        else
            HW_SWITCH(4) = on;
        if (Get_HW_Status(Height_HW) == on && height_status == 0)
        {
            height_status = 1;
        }
        osDelay(10); //�������Ƶ�ʲ���,���Կ���50ms������ˢ��
    }
    memset(swicth_status, err, sizeof(swicth_status)); //��յ�δ��ʼ״̬�����ڱ�Ǵ�ʱ����δ����
    vTaskDelete(Read_Swicth_tasHandle);                //�������б����Ƴ�������
    Read_Swicth_tasHandle = NULL;                      //����ÿ�
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
    if (read_task_exit)
        return err;    //ȷ����ǰ�����ڽ�����
    return SWITCH(id); //���ض�Ӧ���ص�״̬
}

/**********************************************************************
  * @Name    Get_HW_Status
  * @declaration :��ȡָ��ID�ź��⿪�ص�״̬
  * @param   id: [����/��] ���⿪�ر��
  * @retval   : ״̬
  * @author  peach99CPP
***********************************************************************/
int Get_HW_Status(int id)
{
    if (read_task_exit || (id < 1 || id > 4)) //����ֵ���Ʊ������
        return err;
    return HW_SWITCH(id);
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
    read_task_exit = 1; //�˱���Ϊ1 ��ʹ��������ѭ������������
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
    /*��������ʱ�ٶȵı���,���˹��߷����ȶ�*/
    int Switch_Factor = 30;
    int MIN_SPEED = 30;

    if (read_task_exit)
        Start_Read_Switch();

    Set_IMUStatus(false); //�ر�������,��������w�ٶ�������

    short flag1, flag2, x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //���ڲ����Ľ��������ݷ������ж��ٶȵķ��䷽��
        if (dir == 1) //��Y����
        {
            w1 = 1, w2 = 2;
            x_pn = 0, y_pn = 1;
        }
        else if (dir == 2) //��X����
        {
            w1 = 4, w2 = 3;
            x_pn = -1, y_pn = 0;
        }
        else if (dir == 3) //��X����
        {
            w1 = 5, w2 = 6;
            x_pn = 1, y_pn = 0;
        }
        else if (dir == 4) //��Y����
        {
            w1 = 8, w2 = 7;
            x_pn = 0, y_pn = -1;
        }
    }
    //��ʼ����
Closing:
    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0); //����һ�������ٶȣ����ٶ��뷽������й�
    //�ȴ����ض�����
    do
    {
        flag1 = Get_Switch_Status(w1); //��ȡ״̬
        flag2 = Get_Switch_Status(w2);
        /*������һ����䣬ֻ�ڵ������ؿ���ʱ��������*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));

        if (flag1 == err || flag2 == err)
            Start_Read_Switch(); //��ֹ��ʱ����δ�������¿���ѭ��
        //�������
        osDelay(10);

    } while (flag1 == off || flag2 == off); //ֻ����������ͨ�����˳���ѭ��
    osDelay(500);
    if (flag1 == off || flag2 == off)
    {
        MIN_SPEED /= 2; //���͵��ٶ�
        if (ABS(MIN_SPEED) < 5)
            goto switch_exit; //��ֹ����������
        goto Closing;         //�����ص������ĳ���
    }
switch_exit:
    //    Exit_Swicth_Read(); //�����˾͹ر�����
    set_speed(0, 0, 0); //����
    /*******��������Ӧ�ý�һ�����������ǣ����ǻή�ͳ��������ԣ����Բ���ӡ��ڵ��ñ�����֮���Լ�����������*******/
}

/**********************************************************************
  * @Name    Single_Switch
  * @declaration :��ⵥ�߿���
  * @param   switch_id: [����/��]  ���غ� 1-8
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Single_Switch(int switch_id)
{
    //    Set_IMUStatus(false); //ֱ�ӵ���ǽײ���������������ȶ��Ƕ�
    short x, y; //��ͬ������ٶ�����
    short x_vertical, y_vertical;
    int status;         //�洢����״̬�ı���
    if (read_task_exit) //ȷ�����صĿ���״̬
        Start_Read_Switch();
    switch (switch_id) //�����ٶȷ���ʹ�ֱ���ӵ��ٶȷ���
    {
    case 1:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 2:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 3:
        x = 0, y = 1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 4:
        x = 0, y = -1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 5:
        x = 0, y = 1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 6:
        x = 0, y = -1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 7:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    case 8:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    default:
        x = 0, y = 0;
        x_vertical = 0, y_vertical = 0;
    }
RECLOSE:
    while (Get_Switch_Status(switch_id) != on)
    {
        set_speed(x_vertical * VERTICAL, y_vertical * VERTICAL, 0);
        osDelay(5);
    }
    osDelay(500);
    if (Get_Switch_Status(switch_id) != on)
        goto RECLOSE;
    set_speed(x * MIN_ + x_vertical * VERTICAL, y * MIN_ + y_vertical * VERTICAL, 0); //��һ���ٶ�,��������Ҫ�ڴ�ֱ������Ҳ��һ���ٶ�ֵ���⳵������
    do
    {
        status = Get_Switch_Status(switch_id); //��ȡ״̬
        if (status == err)
            Start_Read_Switch(); //��ֹ��ʱ�����˳���������ѭ����
        osDelay(20);             //�������
    } while (status == on);      //ֱ�����ضϿ�����ʱ˵������߽�
    set_speed(0, 0, 0);          //ͣ��
}

void Set_SwitchParam(int main, int vertical)
{
    //�����ٶȵ�API
    MIN_ = main;         //���Ű���ˮƽ������ٶ�
    VERTICAL = vertical; //��ֱ���ӵ��ٶȣ�ȷ�������š�
}
void HWSwitch_Move(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        set_speed(-MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on)
            osDelay(10);
    }
    else if (dir == 2)
    {
        set_speed(MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on)
            osDelay(10);
    }
    set_speed(0, 0, 0);
    osDelay(200);
}

void MV_HW(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        set_speed(-MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on && Get_Stop_Signal() == false)
            osDelay(10);
    }
    else if (dir == 2)
    {
        set_speed(MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on && Get_Stop_Signal() == false)
            osDelay(10);
    }
    set_speed(0, 0, 0);
    osDelay(100);
    Wait_Switches(1);
}
int Get_Height(void)
{
    if (height_status == 0)
    {
        return 1;
    }
    else
    {
        if (Get_HW_Status(Height_HW) == on)
        {
            return 2;
        }
        else 
        {
            return 3;
        }
    }
}
