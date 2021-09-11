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
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
    (void*)read_addr, "u32 read_addr(u32 addr)",
    (void*)write_addr, "void write_addr(u32 addr,u32 val)",
#endif
    (void*)set_speed, "void set_speed(int x, int y, int w)",
    (void*)set_debug_motor,  "void set_debug_motor(int status, int motor_id)",
    (void*)move_by_encoder, "void move_by_encoder(int  direct, int val)",
    (void*)direct_move,  "void direct_move(int direct, int line_num,int edge_if)",
    (void*)set_track_pid,  "void set_track_pid(int kp, int ki, int kd)",
    (void*)track_status, "void track_status(int id, int status)",
    (void*)set_imu_param, "void set_imu_param(int p,int i,int d)",
    (void*)set_imu_status, "void set_imu_status(int status)",
    (void*)Set_InitYaw, "void Set_InitYaw(int target)",
    (void*)turn_angle, "void turn_angle(int mode ,int angle)",
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
    0,	  	//��������
    0,	 	//����ID
    1,		//������ʾ����,0,10����;1,16����
    0,		//��������.bitx:,0,����;1,�ַ���
    0,	  	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
    0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};



















