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
#include "servo.h"

#define Height_HW 5
#define RED_TARGET 1
#define BLUE_TARGET 0

osThreadId Read_Swicth_tasHandle;             //������
void Read_Swicth(void const *argument);       //��������
osThreadId Height_UpadteTask;                 //������
void HeightUpdate_Task(void const *argument); //��������

ScanDir_t Height_Mode = Medium_Head;
Height_t Current_Height = MediumHeight;

Game_Color_t Current_Color = Not_Running;

int MV_Servo_Flag = 1;
int read_task_exit = 1, Height_task_exit = 1; //�����˳���־
static short Height_Flag = 0;

short swicth_status[8]; //����״̬��ֻ���ڲ����и�ֵ
short HW_Switch[10];    //���⿪�ص�״̬
int MIN_ = 60;
int VERTICAL = 5;

#define SWITCH(x) swicth_status[(x)-1] //Ϊ��ֱ���жϿ��ر��
#define HW_SWITCH(X) HW_Switch[(X)-1]

#define Height_SWITCH HW_Switch[4]            //�߶ȵĿ���
#define Side_SWITCH(X) HW_Switch[(X) + 5 - 1] //��ߺ���İ�װλ��

void Judge_Side(int color_mode, int dir)
{
    if (color_mode == Red_)
    {
        if (dir == 5)
        {
            Height_Mode = Medium_Head;
        }
        if (dir == 6)
        {
            Height_Mode = Low_Head;
        }
    }
    else if (color_mode == Blue_)
    {
        if (dir == 5)
        {
            Height_Mode = Low_Head;
        }
        if (dir == 6)
        {
            Height_Mode = Medium_Head;
        }
    }
}
void Start_HeightUpdate(void)
{
    Height_Flag = 0;
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Current_Height = MediumHeight;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityNormal, 0, 128);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}
void HeightUpdate_Task(void const *argument)
{
    Height_Flag = 0;
    while (!Height_task_exit)
    {
        if (Height_Mode == Low_Head)
        {
            Current_Height = LowestHeight;
            while (!Height_task_exit)
            {
                if (Get_HW_Status(Height_HW) == on && Height_Flag == 0)
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                }
                if (Height_Flag == 1 && !Servo_Running)
                {
                    if (Get_HW_Status(Height_HW) == off)
                    {
                        Current_Height = MediumHeight;
                    }
                }
                osDelay(20);
            }
        }
        else if (Height_Mode == Medium_Head)
        {
            Current_Height = MediumHeight;
            while (!Height_task_exit)
            {
                if (Get_HW_Status(Height_HW) == on && Height_Flag == 0)
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                }
                if (Height_Flag == 1 && !Servo_Running)
                {
                    if (Get_HW_Status(Height_HW) == off)
                    {
                        Current_Height = LowestHeight;
                    }
                }
                osDelay(20);
            }
        }
    }
    Current_Height = PrimaryHeight;
    vTaskDelete(Height_UpadteTask);
}
void Exit_Height_Upadte(void)
{
    Height_task_exit = 1;
}
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
        //���⿪�ز���
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
        //���ߺ���
        if (HAL_GPIO_ReadPin(HW_Height_GPIO_Port, HW_Height_Pin) == GPIO_PIN_SET)
            Height_SWITCH = off;
        else
            //��ߺ���
            Height_SWITCH = on;
        if (HAL_GPIO_ReadPin(Side_HW1_GPIO_Port, Side_HW1_Pin) == GPIO_PIN_SET)
            Side_SWITCH(1) = off;
        else
            Side_SWITCH(1) = on;
        if (HAL_GPIO_ReadPin(Side_HW2_GPIO_Port, Side_HW2_Pin) == GPIO_PIN_SET)
            Side_SWITCH(2) = off;
        else
            Side_SWITCH(2) = on;

        osDelay(10); //�������Ƶ�ʲ���,���Կ���10ms������ˢ��
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
    if (read_task_exit || (id < 1 || id > 8)) //����ֵ���Ʊ������
        return err;
    return HW_SWITCH(id);
}

/**********************************************************************
  * @Name    Get_Side_Switch
  * @declaration :��ȡ��߽߱翪�ص�״̬
  * @param   id: [����/��] ���ر��
  * @retval   : ״̬  ɨ���˾�Ϊon
  * @author  peach99CPP
***********************************************************************/
int Get_Side_Switch(int id)
{
    if (id < 1 || id > 2)
        return off;
    return Side_SWITCH(id);
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
  * @Name    Get_Height
  * @declaration : ��ȡ��ʱӦ�õ��õĶ�������
  * @param   None
  * @retval   : ��������
  * @author  peach99CPP
***********************************************************************/
int Get_Height(void)
{
    return Current_Height;
}

/**********************************************************************
  * @Name    MV_HW_Scan
  * @declaration :����MV ��� �����Խ���ƽ̨����ɨ�貢ץȡ
  * @param   dir: [����/��]  �ƶ��ķ���
                 1  Ϊ��ͷ��ǰ   ����ɨ��
                 2  Ϊ��ͷ��ǰ   ����ɨ��
                 5  Ϊ��������   ��ǰ�ƶ�
                 6  Ϊ��������   ����ƶ�
**			 enable_imu: [����/��] �Ƿ�ʹ��IMU�����ֳ���Ƕȵ�Ϊֱ��
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_HW_Scan(int r_b, int dir, int enable_imu)
{
    int time_delay = 0;
    mv_rec_flag = 0;
    Action_Gruop(11, 1);                                    //չ��צ��
    while (Get_Servo_Flag() == false && time_delay <= 5000) //�ȴ���ɣ�ͬʱ���ⳬʱ
    {
        time_delay += 10;
        osDelay(10);
    }
    Set_IMUStatus(enable_imu); //����������״̬
    Judge_Side(r_b, dir);      //�þ��߶��ж�ģʽ
    // mv_rec_flag = 1;           //���������Ե��������¿������ֶ�����һ��
    Start_HeightUpdate();
    /*1 2Ϊ��ǰ���� 5 6Ϊ�Ҳ�*/
    if (dir == 1)
    {
    dir1_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on) //��δ�յ�MVֹͣ�źŻ���⿪������ͨʱ
        {
            set_speed(-MIN_, VERTICAL, 0); //һ����һ������
            osDelay(10);
        }
        set_speed(0, 0, 0);            //ͣ��
        Wait_Switches(1);              //����
        if (Get_HW_Status(dir) == off) //�ж��˳�����ѭ����ԭ������Ǻ��ⴥ���ģ�˵����ʱ����߽磬ɨ��������˳�����
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            //����Ϊ�յ�MVץ����źŶ�ֹͣ���ȵ�������ִ����Ϻ����ɨ�裬ֻ������Ӧ
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1; //��MV����Ŀ��أ�Ϊ��һ��ץ����׼��
            Servo_Running = 0;
            goto dir1_Start_Symbol;
        }
    }
    else if (dir == 2)
    {
        Start_HeightUpdate();
    dir2_Start_Symbol:
        set_speed(MIN_, VERTICAL, 0);
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        Wait_Switches(1);
        if (Get_HW_Status(dir) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {

            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            goto dir2_Start_Symbol;
        }
    }
    else if (dir == 5)
    {
        Start_HeightUpdate();
    dir5_Start_Symbol:
        set_speed(VERTICAL, MIN_, 0);
        while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        if (Get_Side_Switch(1) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            Wait_Switches(3);
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            goto dir5_Start_Symbol;
        }
    }
    else if (dir == 6)
    {
        Start_HeightUpdate();
    dir6_Start_Symbol:
        set_speed(VERTICAL, -MIN_, 0);
        while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        if (Get_Side_Switch(2) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            set_speed(0, 0, 0);
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            Servo_Running = 0;
            goto dir6_Start_Symbol;
        }
    }
}

/**********************************************************************
  * @Name    Get_MV_Servo_Flag
  * @declaration : ״̬�жϽӿ� �����ж��Ƿ����ִ�ж�������飬����ͬʱ�����ظ�ָ��
  * @param   None
  * @retval   : �Ƿ�������ж�����
  * @author  peach99CPP
***********************************************************************/
int Get_MV_Servo_Flag(void)
{
    if (MV_Servo_Flag)
    {
        MV_Servo_Flag = 0;
        return true;
    }
    else
        return false;
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
    int MIN_SPEED = 50;

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
        if (flag1 == err || flag2 == err)
        {
            Start_Read_Switch(); //��ֹ��ʱ����δ�������¿���ѭ��
            continue;
        }
        /*������һ����䣬ֻ�ڵ������ؿ���ʱ��������*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));
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

/**********************************************************************
  * @Name    Set_SwitchParam
  * @declaration : ��������;�ĺ����ӿ�
  * @param   main: [����/��] ��Ҫ�ٶȣ����ű����ƶ����ٶ�
**			 vertical: [����/��]  ��ֱ����ص��ٶȣ���ȷ��������״̬
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void Set_SwitchParam(int main, int vertical)
{
    //�����ٶȵ�API
    MIN_ = main;         //���Ű���ˮƽ������ٶ�
    VERTICAL = vertical; //��ֱ���ӵ��ٶȣ�ȷ�������š�
}

/**********************************************************************
  * @Name    HWSwitch_Move
  * @declaration : ����ʹ�ú������ƶ���ƽ̨��һ��
  * @param   dir: [����/��]  �����ƶ��ķ���
**			 enable_imu: [����/��]  �Ƿ�ʹ��������
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
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
    else if (dir == 5)
    {
        set_speed(VERTICAL, MIN_, 0);
        while (Get_Side_Switch(1) == on)
            osDelay(10);
    }
    else if (dir == 6)
    {
        set_speed(VERTICAL, -MIN_, 0);
        while (Get_Side_Switch(2) == on)
            osDelay(10);
    }
    set_speed(0, 0, 0);
    osDelay(200);
}

/**********************************************************************
  * @Name    MV_HW
  * @declaration :
  * @param   dir: [����/��] ʹ��MV�ͺ�����ɨ�裬�Ѿ���MV_HW_Scan����ȡ��
**			 enable_imu: [����/��]  �Ƿ�ʹ��������
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
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
}
