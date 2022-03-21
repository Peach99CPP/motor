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
#include "general.h"
#include "track_bar_receive.h"
#include "Wait_BackInf.h"

#define Height_HW1 5
#define RED_TARGET 1
#define BLUE_TARGET 0
// 17 18 19��� ��� �м�
osThreadId Read_Swicth_tasHandle;             //������
void Read_Swicth(void const *argument);       //��������
osThreadId Height_UpadteTask;                 //������
void HeightUpdate_Task(void const *argument); //��������

ScanDir_t Height_Mode = Primary_Head;
Height_t Current_Height = PrimaryHeight;
Game_Color_t Current_Color = Not_Running;

int Height_id = 1;
int read_task_exit = 1, Height_task_exit = 1; //�����˳���־
short Height_Flag = 0;

bool QR_Brick = false; //�Ƿ�λ�����ָ߶� ģʽ

short swicth_status[8]; //����״̬��ֻ���ڲ����и�ֵ
short HW_Switch[10];    //���⿪�ص�״̬
int MIN_ = 40;
int VERTICAL = 10;

bool update_finish = true;
bool HeightAvailable = true;

#define Wait_Servo_Done 20000 //�ȴ���������ɵ����ȴ�ʱ��

#define SWITCH(x) swicth_status[(x)-1] //Ϊ��ֱ���жϿ��ر��
#define HW_SWITCH(X) HW_Switch[(X)-1]  // 0��3�±���Ǻ��⿪�ص�λ��

#define Height_SWITCH(x) HW_Switch[(x) + 4 - 1] //�߶ȵĿ��أ���4�͵�5�±������߶Ⱥ���

#define Side_SWITCH(X) HW_Switch[(X) + 6 - 1] //��ߺ���İ�װλ��,�������������±�Ϊ6 ��7

bool Get_HeightAvailable(void)
{
    return HeightAvailable;
}
void Set_HeightAvailable(bool Switch_Status)
{
    HeightAvailable = Switch_Status;
}

void Recover_EnableStatus(void)
{
    Set_MV_Mode(true);
    Set_QR_Status(true);
}

/**********************************************************************
 * @Name    Get_Update_Result
 * @declaration :��ȡ�߶ȸ��µĽ��
 * @param   None
 * @retval   : �Ƿ�������
 * @author  peach99CPP
 ***********************************************************************/
bool Get_Update_Result(void)
{
    return update_finish;
}

/**********************************************************************
 * @Name    Set_Update_Status
 * @declaration : ���ø߶ȸ���ֵ
 * @param   status: [����/��]  ���ú��״̬
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Set_Update_Status(bool status)
{
    update_finish = status;
}

/**********************************************************************
 * @Name    Wait_Update_finish
 * @declaration : �ȴ����½��� �������ס���� ���򲻻�ͣ��
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Update_finish(void)
{
    while (Get_Update_Result() != true)
    {
        set_speed(0, 0, 0);
        osDelay(1);
    }
}

/**********************************************************************
 * @Name    Set_NeedUp
 * @declaration : �����Ƿ�Ϊ��ά��ģʽ
 * @param   if_on: [����/��] �ǻ��
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Set_NeedUp(bool if_on) //�ڴ˴������Ƿ���Ҫ��̬�任�߶�
{
    QR_Brick = if_on;
}
/**********************************************************************
 * @Name    Return_IFQR
 * @declaration : �����Ƿ��ڶ�ά�빤��ģʽ
 * @param   None
 * @retval   : true��Ϊ��ά��ģʽ
 * @author  peach99CPP
 ***********************************************************************/
bool Return_IFQR(void)
{
    return QR_Brick;
}

/**********************************************************************
 * @Name    QR_Mode_Height
 * @declaration : �ڶ�ά��ģʽ��ִ�и߶ȱ任
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void QR_Mode_Height(void)
{
    if (Return_IFQR())
    {
        Inf_Servo_Height(Get_Height()); //���ݸ߶�ִ�ж�����
    }
}
/**********************************************************************
 * @Name    Return_ReverseID
 * @declaration : ��ȡ��ԵĿ��ر��
 * @param   id: [����/��] ��ǰ�Ŀ��ر��
 * @retval   : ͬ�����Ա��
 * @author  peach99CPP
 ***********************************************************************/
int Return_ReverseID(int id)
{
    if (id == 1)
        return 2;
    else if (id == 2)
        return 1;
    else if (id == 5)
        return 6;
    else if (id == 6)
        return 5;
    else
        return 1; //�����������0������������Խ�� todo����ڴ˷�֧����һ�����󱨸�Ĵ�ӡ����
}

void Inf_Servo_Height(int now_height)
{
    if (Return_IFQR()) //�ڸ�������������Ҫ
    {
        static int last_height = PrimaryHeight;
        if (last_height != now_height) //�����ظ�����
        {
            last_height = now_height;
            printf("�߶ȷ����ı�\n");
        }
        Set_Update_Status(false);
        set_speed(0, 0, 0);                 //��ͣ�� todo  �˴�����֮��
        Wait_Servo_Signal(Wait_Servo_Done); //�ȴ���һ���������
        if (now_height == LowestHeight)     //���ݵ�ǰ�߶Ƚ��нǶȵĸ��²���
            Action_Gruop(toLowest, 1);      //���ж�����
        else if (now_height == MediumHeight)
            Action_Gruop(toMedium, 1);
        else if (now_height == HighestHeight)
            Action_Gruop(toHighest, 1);
        else
            osDelay(100);                   //��ʼ״̬
        Wait_Servo_Signal(Wait_Servo_Done); //�ȴ��ź�
        Set_Update_Status(true);
    }
    Set_HeightAvailable(true); //������Ĳ����жԸ߶Ⱥ���Ķ�ȡ��������
}
/**********************************************************************
 * @Name    Judge_Side
 * @declaration :
 * @param   color_mode: [����/��]
 **			 dir: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Judge_Side(int dir)
{
    if (dir == 5)
    {
        Current_Height = MediumHeight;
    }
    else if (dir == 6)
    {
        Current_Height = LowestHeight;
    }
}

/**********************************************************************
 * @Name    Start_HeightUpdate
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Start_HeightUpdate(void)
{
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Height_Flag = 0;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityHigh, 0, 256);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}

/**********************************************************************
 * @Name    HeightUpdate_Task
 * @declaration :
 * @param   argument: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void HeightUpdate_Task(void const *argument)
{
    Height_Flag = 0;
    static short temp_roi_chnage_flag = 1;
    // todo �������Ӳ�������������ֵ�ı�����ָ��ʹ���ĸ����⣨ʹ��˫�߶Ⱥ��������£�
    while (!Height_task_exit)
    {
        if (Current_Height == LowestHeight)
        {
            Height_id = 2; //ʹ��2�����жϸ߶�ģʽ /
            // todo �˴��ǵü���Ƿ��Ѿ����úö�Ӧ�߶ȵ��л�������
            printf("\n************Current Height:LowestHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_Servo_Flag() == true && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                    printf("\n************Current Height:HighestHeight***************\n");
                    Inf_Servo_Height(Current_Height);
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        Current_Height = MediumHeight;
                        Height_Flag = 2;
                        printf("\n************Current Height:MediumHeight***************\n");
                        Inf_Servo_Height(Current_Height);
                    }
                }
                osDelay(1);
            }
        }
        else if (Current_Height == MediumHeight)
        {
            Height_id = 1; // todo��ʱ�޸�
            OpenMV_ChangeRoi(1);
            printf("\n************Current Height:MediumHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Servo_Flag() == true && Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Height_id = 1; // todo �˴���Ҫ����£���Ϊһ��ʼʹ������ʱ�ĺ�����
                    Current_Height = HighestHeight;
                    Inf_Servo_Height(Current_Height);
                    printf("\n************Current Height:HighestHeight***************\n");
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        Current_Height = LowestHeight;
                        Height_Flag = 2;
                        Inf_Servo_Height(Current_Height);
                        printf("\n************Current Height:LowestHeight***************\n");
                    }
                    //�����ڽ��ն˾���������
                    if (Get_Height_Switch(2) == on && temp_roi_chnage_flag == 1)
                    {
                        temp_roi_chnage_flag = 0;
                        OpenMV_ChangeRoi(2);
                    }
                }

                osDelay(1);
            }
        }
        osDelay(5);
    }
    Height_Flag = 0;
    Current_Height = PrimaryHeight;
    vTaskDelete(NULL);
}

/**********************************************************************
 * @Name    Exit_Height_Upadte
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
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
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityAboveNormal, 0, 128);
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
        if (HAL_GPIO_ReadPin(HW_Height1_GPIO_Port, HW_Height1_Pin) == GPIO_PIN_SET)
            Height_SWITCH(1) = off;
        else
            //��ߺ���
            Height_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_Height2_GPIO_Port, HW_Height2_Pin) == GPIO_PIN_SET)
            Height_SWITCH(2) = off;
        else
            //��ߺ���
            Height_SWITCH(2) = on;

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
    vTaskDelete(NULL);                                 //�������б����Ƴ�������
    Read_Swicth_tasHandle = NULL;                      //����ÿ�
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
 * @Name    Get_Switch_Status
 * @declaration : ��ȡָ��ID���ص�ͨ��״̬
 * @param   id: [����/��] �����ţ�1-8��
 * @retval   : �ÿ��ص�ͨ��״̬
 * @author  peach99CPP
 ***********************************************************************/
int Get_Switch_Status(int id)
{
    if (read_task_exit)
        return err; //ȷ����ǰ�����ڽ�����
    if (id < 1 || id > 10)
        return off; // todo���п�����������ʱ�ǵ��޸Ĵ˴���ֵ
    //����������Ϊ�˱��������ж�����
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
    // todo���п�����������ʱ�ǵ��޸Ĵ˴���ֵ
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
    if (id < 0 || id > 3) // todo���п�����������ʱ�ǵ��޸Ĵ˴���ֵ
        return off;
    return Side_SWITCH(id);
}
int Get_Height_Switch(int id)
{
    if (id < 1 || id > 2)
        return off; // todo �ǵ��ڸ���Ԫ������������´δ˴������Ʒ�Χ
    return Height_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Height
 * @declaration : ��ȡ��ʱ�߶��Լ���Ӧ�õ��õĶ�������
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
                 1  Ϊ��ͷ��ǰ   ����ɨ��   ��������ƽ̨
                 2  Ϊ��ͷ��ǰ   ����ɨ��   ��������ƽ̨
                 5  Ϊ��������   ��ǰ�ƶ�   ���ڽ���ƽ̨
                 6  Ϊ��������   ����ƶ�   ���ڽ���ƽ̨
**			 enable_imu: [����/��] �Ƿ�ʹ��IMU�����ֳ���Ƕȵ�Ϊֱ��
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void MV_HW_Scan(int color, int dir, int enable_imu)
{
    Set_NeedUp(false);    //��ֹ�߶ȱ任
    MV_Start();           //����Openmv
    Disable_StopSignal(); //��ʱ����ͣ��

    Set_IMUStatus(enable_imu); //����������״̬
    if (dir == 5 || dir == 6)  //����ƽ̨����
    {
        osDelay(100);
        MV_SendCmd(1, color);    //��openmv������ɫ�ź�  todo�ǵøĻ�1
        Action_Gruop(11, 1);     //չ��צ��
        Set_IFUP(true);          //���û�е�۶�����־λ
        Judge_Side(dir);         //�þ��߶��ж�ģʽ�����ݲ�����������ֵ ֻ�ڽ���ƽ̨��Ҫ�õ�
        Wait_Servo_Signal(5000); //�ȴ�����ź�
        Start_HeightUpdate();    //��ʼ�߶��ж�ģʽ
    }
    else if (dir == 1 || dir == 2) //����ƽ̨
    {
        Openmv_Scan_Bar(1, color); //��OPENMV��ʼִ������ƽ̨����
        Wait_Servo_Signal(5000);
    }
    /*1 2Ϊ��ǰ���� 5 6Ϊ���Ҳ�*/
    if (dir == 1) //ʹ����ߵĺ������������
    {
        while (Get_HW_Status(dir) == false)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5); //���ƶ�����ʱ���⿪ʼɨ�赽����ֹ�·�ֱ�ӱ�����
        }
        set_speed(0, 0, 0);
    dir1_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on) //��δ�յ�MVֹͣ�źŻ���⿪������ͨʱ
        {
            if (Get_HW_Status(Return_ReverseID(dir)) == on) //�鿴�Բ���⿪�ص�״̬
                set_speed(MIN_ * 0.8, VERTICAL, 0);         //һ����һ������
            else
                set_speed(MIN_ * 0.8, 0, 0); //��ʱ�Ѿ���һ�߳�ȥ����ֹ���ؿ�����ȡ����ֱ������ٶȣ�����ˮƽ���ٶȼ���
            osDelay(5);
        }
        set_speed(0, 0, 0);            //ͣ��
        if (Get_HW_Status(dir) == off) //�ж��˳�����ѭ����ԭ������Ǻ��ⴥ���ģ�˵����ʱ����߽磬ɨ��������˳�����
        {
            Openmv_Scan_Bar(0, 1);
            Exit_Height_Upadte();
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            //����Ϊ�յ�MVץ����źŶ�ֹͣ���ȵ�������ִ����Ϻ����ɨ�裬ֻ������Ӧ
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
            osDelay(100);         //ûɶ�ã�����İ�
            goto dir1_Start_Symbol;
        }
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir) == false)
        {
            set_speed(-MIN_, 0, 0); //�����ƶ�
            osDelay(5);             //���ƶ�����ʱ���⿪ʼɨ�赽����ֹ�·�ֱ�ӱ�����
        }
        set_speed(0, 0, 0);
    dir2_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on)
        {
            if (Get_HW_Status(Return_ReverseID(dir)) == on) //�鿴�Բ���⿪�ص�״̬
                set_speed(-MIN_ * 0.8, VERTICAL, 0);        //һ����һ������
            else
                set_speed(-MIN_ * 0.8, 0, 0); //��ʱ�Ѿ���һ�߳�ȥ����ֹ���ؿ�����ȡ����ֱ������ٶȣ�����ˮƽ���ٶȼ���
            osDelay(5);
        }
        set_speed(0, 0, 0);
        if (Get_HW_Status(dir) == off)
        {
            Openmv_Scan_Bar(0, 1);
            Exit_Height_Upadte();
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
            osDelay(100);         //ûɶ�ã�����İ�
            goto dir2_Start_Symbol;
        }
    }
    //ֻ���ڽ���ƽ̨�ƶ�ʱ��Ҫʹ�ø߶���Ϣ��ǰ����������ɨ�����ƽ̨
    else if (dir == 5)
    {
        while (Get_Side_Switch(1) == off)
        {
            set_speed(0, MIN_, 0);
            osDelay(5); //���⿪�ؿ���
        }
    dir5_Start_Symbol:

        while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
        {
            set_speed(VERTICAL, MIN_ * 0.5, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        if (Get_Side_Switch(1) == off)
        {
            Action_Gruop(4, 1); //�����е��
            Set_IFUP(false);
            Exit_Height_Upadte();
            MV_Stop(); //ֹͣ������Ӧ
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
            Recover_EnableStatus();
            osDelay(100); //ûɶ�ã�����İ�
            goto dir5_Start_Symbol;
        }
    }
    else if (dir == 6)
    {
        while (Get_Side_Switch(2) == off)
        {
            set_speed(0, -MIN_, 0);
            osDelay(5); //���⿪�ؿ���
        }
    dir6_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on) //Ϊ�˶���һ��·�̣���ʱ��Ϊ1��
        {
            set_speed(VERTICAL * 1.5, -MIN_ * 0.5, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);            //ͣ����˵
        if (Get_Side_Switch(2) == off) //�Ǳ�Ե�ĺ��⵼�µ�ͣ��
        {
            Action_Gruop(4, 1); //�����е��
            Set_IFUP(false);
            Exit_Height_Upadte(); //��������
            MV_Stop();            //ֹͣ������Ӧ
            Wait_Servo_Signal(Wait_Servo_Done);
            return; //�˳�
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
            Recover_EnableStatus();
            osDelay(100);           //ûɶ�ã�����İ�
            goto dir6_Start_Symbol; //�ص���ͷλ��
        }
    }
}

/**
 * @description: ��Ը����ľ��κͶ�ά��ĺ���
 * @param {int} dir  ���з��� 5Ϊ������ 6Ϊ���ҵ���
 * @param {int} color  Ŀ�����ɫ 1��2��
 * @param {int} QR   �Ƿ�������
 * @param {int} imu_enable  �Ƿ����������Ǹ����Ƕ�ά��
 * @return {*} null
 */
void Brick_QR_Mode(int dir, int color, int QR, int imu_enable)
{
    Set_NeedUp(QR); //�߶ȱ任������ر�
    QR_Mode_Init(QR, (QRcolor_t)color);
    MV_Start();                //����Openmv
    Disable_StopSignal();      //��ʱ����ͣ��
    osDelay(300);              //�ȴ�����������
    Recover_EnableStatus();    //����mv�Ͷ�ά�빤��
    MV_SendCmd(5, color);      //����ģʽ��ɨ����
    Set_IMUStatus(imu_enable); //���������ǿ���
    if (dir == 5 || dir == 6)
    {
        Action_Gruop(14, 1);     //����
        Set_IFUP(true);          //��־λ
        Judge_Side(dir);         //���ݷ���������ʼ�߶ȶԻ�е�۶�������б任
        Wait_Servo_Signal(5000); //�ȴ����������ź� ���ĵȴ�5s
        QR_Mode_Height();        //����ģʽ���и߶ȱ任 ��ʱ��Ŀ��߶�����judge_side�ĸ�ֵ
        Start_HeightUpdate();    //��Ϊ�����Լ�ִ�и߶ȱ任 ���Բ���Ҫ������ͷִ�и߶ȱ任
        if (dir == 5)
        {
            while (Get_Side_Switch(1) == off)
            {
                set_speed(0, MIN_*0.5, 0); //ȷ����ʱ���صĹ���״̬����
                osDelay(5);            //���⿪�ؿ���
            }
        dir5_Start_Symbol:
            // ����֮ ����߶ȸ���δ��� ���� ͣ��������� ���� ��ʱ�Ѿ��߳����򣨺��⿪��off��
            while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on && Get_Update_Result())
            {
                set_speed(VERTICAL, MIN_, 0);
                osDelay(5);
            }
            set_speed(0, 0, 0);
            if (Get_Side_Switch(1) == off)
            {
                Action_Gruop(4, 1); //�����е��
                Set_IFUP(false);
                Exit_Height_Upadte();
                MV_Stop(); //ֹͣ������Ӧ
                Wait_Servo_Signal(Wait_Servo_Done);
                return;
            }
            else if (Get_Update_Result() == false)
            {
                Wait_Update_finish();
                osDelay(100);
                goto dir5_Start_Symbol; //�ص���ͷλ��
            }
            else
            {
                Wait_Servo_Signal(Wait_Servo_Done);
                QR_Mode_Height();
                Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
                Recover_EnableStatus();
                osDelay(100); //ûɶ�ã�����İ�
                goto dir5_Start_Symbol;
            }
        }
        else if (dir == 6)
        {
            while (Get_Side_Switch(2) == off)
            {
                set_speed(0, -MIN_*0.5, 0);
                osDelay(5); //���⿪�ؿ���
            }
        dir6_Start_Symbol:
            while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on && Get_Update_Result()) //Ϊ�˶���һ��·�̣���ʱ��Ϊ1��
            {
                set_speed(VERTICAL, -MIN_ * 0.7, 0);
                osDelay(5);
            }
            set_speed(0, 0, 0);            //ͣ����˵
            if (Get_Side_Switch(2) == off) //�Ǳ�Ե�ĺ��⵼�µ�ͣ��
            {
                Action_Gruop(4, 1); //�����е��
                Set_IFUP(false);
                Exit_Height_Upadte(); //��������
                MV_Stop();            //ֹͣ������Ӧ
                Wait_Servo_Signal(Wait_Servo_Done);
                return; //�˳�
            }
            else if (Get_Update_Result() == false)
            {
                Wait_Update_finish();
                osDelay(100);
                goto dir6_Start_Symbol; //�ص���ͷλ��
            }
            else
            {
                Wait_Servo_Signal(Wait_Servo_Done);
                QR_Mode_Height();
                Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
                Recover_EnableStatus();
                osDelay(100);           //ûɶ�ã�����İ�
                goto dir6_Start_Symbol; //�ص���ͷλ��
            }
        }
    }
    MV_Stop(); //�ر�MV�����й��� �����ʱɨ���ֿ������ϴ�������
}
/**
 * @name: Kiss_Ass
 * @brief: ����ʹ�� ƨ�ɶ�׼��
 * @param {int} dir ����
 * @param {int} enable_imu ������ʹ�ܿ���
 * @return {*}
 */
void Kiss_Ass(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu); //����һ�²����������ǵ�����״̬
    if (dir == 1)
    {
        while (Get_HW_Status(dir + 2) == off)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir + 2) == on)
        {
            set_speed(-MIN_ * 2, 0, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        // TODO �ڴ�ʱ���е���ȥ�Ķ����飬�Ѷ�������ȥF
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir + 2) == off)
        {
            set_speed(-MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir + 2) == on)
        {
            set_speed(MIN_ * 2, 0, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        // TODO �ڴ�ʱ���е���ȥ�Ķ����飬�Ѷ�������ȥ
    }
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
    int Switch_Factor = 80;
    int MIN_SPEED = 100;

    if (read_task_exit)
        Start_Read_Switch();

    track_status(1, 0); //�ر�ѭ���棬������ɷ����ϵ�Ӱ��
    track_status(2, 0);

    volatile static short flag1, flag2;
    short x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //���ڲ����Ľ��������ݷ������ж��ٶȵķ��䷽��
        if (dir == 1) //��Y����
        {
            w1 = 1, w2 = 2;
            x_pn = 0, y_pn = 1;
        }
        /***������Ѿ�������
        // else if (dir == 2) //��X����
        // {
        //     w1 = 4, w2 = 3;
        //     x_pn = -1, y_pn = 0;
        *****/
        else if (dir == 3) //��X����
        {
            w1 = 3, w2 = 4;
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
    Set_IMUStatus(false);
    flag1 = flag2 = 0;
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
        if (flag1 == on || flag2 == on) //���п��ش�����ʱ���ر�������
            Set_IMUStatus(false);       //�ر�������,��������w�ٶ�������
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
    Set_InitYaw(0);
    Set_IMUStatus(true);
    set_speed(0, 0, 0); //����
    /*******��������Ӧ�ý�һ�����������ǣ����ǻή�ͳ��������ԣ����Բ���ӡ��ڵ��ñ�����֮���Լ�����������*******/
    // todo�������꺯������ʵ����Ҫ���������ǽǶȵ�����
}

/**********************************************************************
 * @Name    HWSwitch_Move
 * @brief  ����ʹ�ú������ƶ���ƽ̨��һ��
 * @param   dir: [����/��]  �����ƶ��ķ��� 1 2 Ϊ���� 5 6Ϊ��ߵ� ����ο�����ƽ̨
 **			 enable_imu: [����/��]  �Ƿ�ʹ��������
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void HWSwitch_Move(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        while (Get_HW_Status(dir) == off)
        {
            set_speed(-MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir) == on)
        {
            set_speed(-MIN_, VERTICAL, 0);
            osDelay(10);
        }
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir) == off)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir) == on)
        {
            set_speed(MIN_, VERTICAL, 0);
            osDelay(10);
        }
    }
    else if (dir == 5)
    {
        while (Get_Side_Switch(1) == off)
        {
            set_speed(0, MIN_, 0);
            osDelay(5);
        }
        while (Get_Side_Switch(1) == on)
        {
            set_speed(VERTICAL, MIN_, 0);
            osDelay(10);
        }
    }
    else if (dir == 6)
    {
        while (Get_Side_Switch(2) == off)
        {
            set_speed(0, -MIN_, 0);
            osDelay(5);
        }
        while (Get_Side_Switch(2) == on)
        {
            set_speed(VERTICAL, -MIN_, 0);
            osDelay(10);
        }
    }
    set_speed(0, 0, 0);
    osDelay(100);
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
 * @Name    QR_Scan
 * @declaration :ʹ�ö�ά����н���ƽ̨��ɨ��
 * @param   status: [����/��]  �Ƿ���
 **			 color: [����/��]  Ҫץ����ɫ�������жϸ߶�  �����ɫ 2������ɫ
 **			 dir: [����/��]    ����
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void QR_Scan(int status, int color, int dir, int enable_imu)
{
#define OPEN_Claws 1      //צ�Ӷ�Ӧ�Ķ������צ���ſ�һ�����Ӱ���ά��ɨ��
    int time_delay = 0;   //���ⳬʱ��������ʱ����
    Disable_StopSignal(); //���Կ�����
    if (status)           //ʹ��״̬
    {
        Set_QR_Status(true);                                    //����������mv�Ͷ�ά��Ĺ���״̬
        Set_IMUStatus(enable_imu);                              //�����ԣ���������ʱ���׵��¿���
        Set_QR_Target((QRcolor_t)color);                        //����Ŀ��
        Judge_Side(dir);                                        //���ݷ����ж���ʼ�߶�
        Action_Gruop(11, 1);                                    //����צ��
        while (Get_Servo_Flag() == false && time_delay <= 5000) //�ȴ���ɣ�ͬʱ���ⳬʱ��������5���ʱ����ֵ
        {
            time_delay += 10;
            osDelay(10);
        }
        time_delay = 0;       //���³�ʼ������
        Set_QR_Status(1);     //��ʼ�Զ�ά�����Ϣ��ȡ�з�Ӧ
        Start_HeightUpdate(); //��ʼ���¸߶���Ϣ������

        if (dir == 5)
        {
            while (Get_Side_Switch(1) == off) //���⿨��
            {
                set_speed(0, MIN_, 0); //�ȸ�һ���ٶ�һֱ�� �����ʼʱ����ֱ��û����������ֱ�ӽ���
                osDelay(5);
            }
        QR_Scan5_Symbol:
            set_speed(VERTICAL, MIN_, 0); //��һ��ƽ�кʹ�ֱ���ٶȣ�һ�߽�����һ���ƶ�
            while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
                osDelay(5);
            set_speed(0, 0, 0);
            if (Get_Side_Switch(1) == off)
            {
                Action_Gruop(4, 1); //�����е��
                Set_QR_Status(0);
                Exit_Height_Upadte();
                return;
            }
            else
            {
                while (Get_Servo_Flag() == false)
                    osDelay(5);
                Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
                osDelay(100);         //ûɶ�ã�����İ�
                goto QR_Scan5_Symbol;
            }
        }
        else if (dir == 6)
        {
            while (Get_Side_Switch(2) == off)
            {
                set_speed(0, -MIN_, 0);
                osDelay(5); //���⿪�ؿ���
            }
        QR_Scan6_Symbol:
            set_speed(VERTICAL, -MIN_, 0);
            while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on)
                osDelay(5);
            set_speed(0, 0, 0);
            if (Get_Side_Switch(2) == off)
            {
                Action_Gruop(4, 1); //�����е��
                Set_QR_Status(0);   //�����󴥷�
                Exit_Height_Upadte();
                return;
            }
            else
            {
                while (Get_Servo_Flag() == false)
                    osDelay(5);
                Disable_StopSignal(); //���ͣ����־λ����ʱ���Կ���
                osDelay(100);         //ûɶ�ã�����İ�
                goto QR_Scan6_Symbol;
            }
        }
    }
    else
    {
        Set_QR_Status(0);                                       //�رն�ά�����Ϣ����
        Action_Gruop(4, 1);                                     //�����е��
        while (Get_Servo_Flag() == false && time_delay <= 5000) //�ȴ���ɣ�ͬʱ���ⳬʱ��������5���ʱ����ֵ
        {
            time_delay += 10;
            osDelay(10);
        }
    }
}
/**
 * @name:
 * @brief:
 * @param {*}
 * @return {*}
 */
void Ring_Move(void)
{
#define Ring_Action 26                      //��Ӧ������ı��
#define Ring_HW_ID                          //���õĺ���ı��
#define Side_Factor 0.3                     //Ϊ���ַ����һ����ֱ�ٶ�
    set_speed(MIN_ * Side_Factor, MIN_, 0); //��ʼ������
    while (Get_Side_Switch(1) == off)       //���ø߶ȵĺ������λ�õ��ж�
        osDelay(1);                         //��ϵͳ����ʱ��
    set_speed(0, 0, 0);                     //����λ������ͣ��
    osDelay(1000);                          //��ʱ�����ڻ���
    // Action_Gruop(Ring_Action, 1);           //ִ�ж�Ӧ�Ķ�����
}
void Disc_Mea(void)
{
    MV_Stop();
    Action_Gruop(25, 1);     //ִ��Ԥ��������
    Wait_Servo_Signal(2000); //�ȴ�������
    printf("\n��ʼ����Բ�̻�\n");
    Wait_Switches(1);    //ײ�����������Ƕ��Լ���λ
    Action_Gruop(35, 1); //ִ�еڶ���������
    printf("\n��ʼɨ��\n");
    Wait_Servo_Signal(2000);      //�ȴ����������
    MV_Start();                   //����openmv
    MV_SendCmd(6, 1);             // 1��2�� 3��
    while (Get_DiscStatus() == 0) //�ȵ�9��Ŀ����ץ��
        ;
    printf("\n��ʼ����Ŀ������ɫ\n");
    Action_Gruop(33, 1);     //ִ���л��ֿ⶯����
    Wait_Servo_Signal(2000); //�ȴ����������
    MV_SendCmd(6, 3);        //֪ͨMV�л�Ŀ����ɫ
    while (Get_DiscStatus() == 1)
        ;
    printf("\n����\n");
    Action_Gruop(24, 1);     //�����Ķ�����
    Wait_Servo_Signal(2000); //�ȴ����������
    MV_Stop();
}
