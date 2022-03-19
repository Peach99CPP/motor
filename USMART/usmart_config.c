#include "usmart.h"
#include "usmart_str.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����)
#include "delay.h"
#include "motor.h"
#include "chassis.h"
#include "imu_pid.h"
#include "track_bar_receive.h"
#include "chassis_control.h"
#include "openmv.h"
#include "servo.h"
#include "atk_imu.h"
#include "read_status.h"
#include "servo.h"
#include "general.h"
#include"avoid_obs.h"
/****��������ģʽ�Ŀ���*****/

#define DEBUG_MOTOR 0   //���Ե��
#define DEBUG_TRACKER 0 //����ѭ����
#define DEBUG_IMU 1     //����������
#define DEBUG_CHASSIS 1 //�����˶�
#define DEBUG_SWITCH 1  //�ᴥ���ء����⿪��
#define DEBUG_OPENMV 1  // openmvͨѶ
#define Debug_Servo 1   //���ͨѶ
extern void Global_Debug(void);
/******����ʾ�����ø�������****/
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[] =
    {
#if USMART_USE_WRFUNS == 1 //���ʹ���˶�д����
        (void *)read_addr,
        "u32 read_addr(u32 addr)",
        (void *)write_addr,
        "void write_addr(u32 addr,u32 val)",
#endif
#if DEBUG_MOTOR == 1
        (void *)set_debug_motor,
        "void set_debug_motor(int status, int motor_id)",
        (void *)set_debug_speed,
        "void set_debug_speed(int speed)",
        (void *)set_motor_pid,
        "void set_motor_pid(int kp, int ki, int kd)",
        (void *)set_motor_maxparam,
        "void set_motor_maxparam(int integrate_max, int control_output_limit)",
#endif
#if DEBUG_TRACKER == 1
        (void *)set_track_pid,
        "void set_track_pid(int kp, int ki, int kd)",
        (void *)track_status,
        "void track_status(int id, int status)",
#endif
#if DEBUG_IMU == 1
        (void *)set_imu_param,
        "void set_imu_param(int p,int i,int d)",
        (void *)set_imu_status,
        "void set_imu_status(int status)",

/**�����ǲ���**/
#endif
#if DEBUG_OPENMV == 1
        (void *)Openmv_Scan_Bar,
        "void Openmv_Scan_Bar(int status,int color)",
        (void *)MV_SendCmd,
        "void MV_SendCmd(const uint8_t event_id, const int param)",
#endif
#if DEBUG_CHASSIS == 1
        /**�����˶�����**/
        (void *)Comfirm_Online,
        "void Comfirm_Online(int dir)",
        (void *)move_slantly,
        "void move_slantly(int dir, int speed, uint16_t delay)",
        /***Ŀǰ�ò���
    (void *)set_motor,
    "void set_motor(int motor_id, int control_val)",
    ***/
        (void *)set_speed,
        "void set_speed(int x, int y, int w)",
        (void *)move_by_encoder,
        "void move_by_encoder(int  direct, int val)",
        (void *)direct_move,
        "void direct_move(int direct, int line_num, int edge_if,int imu_if)",
#endif
#if DEBUG_SWITCH == 1
        /***�ᴥ���ز���***/
        (void *)Wait_Switches,
        "void Wait_Switches(int dir)",
        //    (void *)Single_Switch,
        //    "void Single_Switch(int switch_id)",
        //    (void *)Set_SwitchParam,
        //    "void Set_SwitchParam(int main,int vertical)",
        (void *)HWSwitch_Move,
        "void HWSwitch_Move(int dir,int enable_imu)",
        (void *)MV_HW_Scan,
        "void MV_HW_Scan(int color, int dir, int enable_imu)",
        (void *)Brick_QR_Mode,
        "void Brick_QR_Mode(int dir, int color, int QR, int imu_enable)",
        (void *)QR_Scan,
        "void QR_Scan(int status, int color, int dir, int enable_imu)",
        (void *)Kiss_Ass,
        "void Kiss_Ass(int dir,int enable_imu)",
        (void*)Ring_Move,
        "void Ring_Move(void)",
#endif
#if Debug_Servo == 1

        (void *)Ass_Door,
        "void Ass_Door(int status)",
        (void *)Lateral_infrared,
        "void Lateral_infrared(int status)",
        (void *)Baffle_Control,
        "void Baffle_Control(int up_dowm)",
        (void *)Single_Control,
        "void Single_Control(int id, int control_mode, int angle, int  time, int delay)",
        (void *)Action_Gruop,
        "void Action_Gruop(int id, int  times)",
#endif
        /****��������,һֱ�Ǵ򿪵�***/
        (void *)set_imu_status,
        "void set_imu_status(int status)",
        (void *)Set_InitYaw,
        "void Set_InitYaw(int target)",
        (void *)Turn_angle,
        "Turn_angle(int mode, int angle, int track_enabled)",
        (void *)Wait_For_Avoid,
        "void Wait_For_Avoid(int dir)",
        (void *)Global_Debug,
        "void Global_Debug(void)",

};
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev =
    {
        usmart_nametab,
        usmart_init,
        usmart_cmd_rec,
        usmart_exe,
        usmart_scan,
        sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), //��������
        0,                                                         //��������
        0,                                                         //����ID
        1,                                                         //������ʾ����,0,10����;1,16����
        0,                                                         //��������.bitx:,0,����;1,�ַ���
        0,                                                         //ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
        0,                                                         //�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};
