#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
#include "avoid_obs.h"
#include "atk_imu.h"
#include "track_bar_receive.h"
#include "servo.h"
#include "openmv.h"
#define DUBUG_MOTOR 0
int if_OsRunning(void);
void Set_OSRunningFlag(int status);

// todo  ����ǰ���ü��
#define Blue_Route 1
#define Red_Route 0
#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
// todo �����ﶨ������볡
int Os_RunningFlag = 0;
uint8_t cmd[3] = {0xff, 0x00, 0x99};
void Startdebug(void const *argument)
{
    Set_OSRunningFlag(true);
    // MV_Start();// todo�ڴ˴���MV�ر� ������ָ�� ��������������
    Start_Read_Switch(); //������ȡ����״̬������
    avoid_keep();        //��ʼ��ȡ�������źŵ�����
    osDelay(100);        //�ӳ�һ�� �ȴ�ϵͳ�ȶ�
#if DUBUG_MOTOR == 1
    track_status(1, 0);
    track_status(2, 0);
    Set_IMUStatus(false);
#endif

    while (1)
    {
#if DUBUG_MOTOR == 1
        motor_debug();
#endif
        osDelay(1);
    }
}

void Global_Debug(void)
{
#if Blue_Route == 1
    move_by_encoder(2, 30);                   //��ǰ��һ�����ȷ��ǰ��ѭ�������
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ������������
    direct_move(2, 3, 0, 1);                  //��ǰֱ���������� todo ���޸�
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);    //�ȴ��������
    printf("\r\n�������\r\n");               //������Ϣ
    Wait_Switches(1);
    printf("�������\r\n");
    Set_InitYaw(0);
    HWSwitch_Move(1, 1);
    printf("��������\r\n");
    MV_HW_Scan(1, 2, 1);
    printf("�������\r\n");

    Turn_angle(1, 180, 1);
    direct_move(2, 1, 0, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    direct_move(1, 1, 1, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    move_slantly(1, 120, 1200);
    Wait_Switches(3);
    Set_InitYaw(180);
    HWSwitch_Move(5, 1);
    MV_HW_Scan(1, 6, 1);
    Brick_QR_Mode(5, 1, 0, 1);
    move_slantly(3, 150, 1600);
    Turn_angle(1, -90, 1);
    osDelay(100);
    direct_move(2, -1, 0, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    Wait_Switches(4);
    Set_InitYaw(-90);

    // Action_Gruop(13, 1); todo���⼸���������з�װ
    // osDelay(1000);
    // Action_Gruop(8, 1);
    // osDelay(1000);
    // Action_Gruop(12, 1);
    // osDelay(1000);
    // Action_Gruop(7, 1);
    // osDelay(1000);
    Lateral_infrared(1);                      //�򿪲������
    Kiss_Ass(1, 1);                           //��λ��ȥ
    Ass_Door(1);                              //��ƨ�ɺ������ ����
    osDelay(5000);                            //ȷ���������
    Ass_Door(0);                              //���Թ���
    move_by_encoder(2, 10);                   //��ǰ��һ��
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    Turn_angle(1, 180, 0);
    Wait_Switches(1);
    Set_InitYaw(90);
    HWSwitch_Move(2, 1);
    move_by_encoder(1, -12);
    Wait_Switches(1);
    move_by_encoder(2, -78);
    Turn_angle(1, -90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
#elif RED_Route == 1
    move_by_encoder(2, 30);
    direct_move(2, 3, 0, 1);
    Wait_Switches(1);
    Set_InitYaw(0);
    HWSwitch_Move(1, 1);
    MV_HW_Scan(1, 1, 1);
    direct_move(2, -2, 1, 1);
    move_slantly(4, 120, 1500);
    Wait_Switches(3);
    Set_InitYaw(0);
    HWSwitch_Move(5, 1);
    MV_HW_Scan(1, 6, 1);
    Brick_QR_Mode(5, 1, 0, 1);
    move_slantly(3, 150, 1800);
    Turn_angle(1, -90, 0);
    direct_move(2, -1, 0, 1);
    Wait_Switches(4);
    Set_InitYaw(-90);
    // todo���������ӿ����ŵĶ���������
    move_by_encoder(2, 10);
    Turn_angle(1, 180, 0);
    Wait_Switches(1);
    Set_InitYaw(90);
    HWSwitch_Move(2, 1);
    move_by_encoder(1, -12);
    Wait_Switches(1);
    move_by_encoder(2, -78);
    Turn_angle(1, 90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
#endif
}
void Go_Home(int color)
{
    if (color != 1 && color != 2)
        return;
    move_by_encoder(2, -80);
    if (color == 1)
        Turn_angle(1, 90, 0);
    else
        Turn_angle(1, -90, 0);
    direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
}
int if_OsRunning(void)
{
    return Os_RunningFlag;
}
void Set_OSRunningFlag(int status)
{
    Os_RunningFlag = status;
}

void Goto_Warehouse(void)
{
    ;
}

osThreadId GameTaskHandle = NULL;            //������
void GameTaskTaskFunc(void const *argument); //����ʵ�ֺ���
bool GameTaskTask_Exit = 1;                  //�Ƿ��˳�

void Game_On(void)
{
    if (GameTaskTask_Exit && GameTaskHandle == NULL)
    {
        GameTaskTask_Exit = 0;
        osThreadDef(GameTask, GameTaskTaskFunc, osPriorityHigh, 0, 1024); //��������ṹ��
        GameTaskHandle = osThreadCreate(osThread(GameTask), NULL);        //��������
    }
}
void GameTaskTaskFunc(void const *argument)
{
    while (!GameTaskTask_Exit)
    {
        Global_Debug();
    }
    GameTaskHandle = NULL;
    vTaskDelete(NULL);
}
