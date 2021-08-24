#include "usmart.h"
#include "usmart_str.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����)
#include "delay.h"
#include "motor.h"
#include "chassis.h"
#include "imu.h"
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
    (void*)read_addr, "u32 read_addr(u32 addr)",
    (void*)write_addr, "void write_addr(u32 addr,u32 val)",
#endif
    (void*)read_encoder, "float read_encoder(int motor_id)",
    (void*)set_motor, "void set_motor(int motor_id, int control_val)",
    (void*)set_motor_pid, "void set_motor_pid(int kp, int ki, int kd)",
    (void*)set_motor_maxparam, "void set_motor_maxparam(int integrate_max, int control_output_limit)",
    (void*)clear_all_speed, "void clear_all_speed(void)",
    (void*)set_speed, "void set_speed(int x, int y, int w)",
    (void*)set_debug_speed, "void set_debug_speed(int speed)",
    (void*)set_debug_motor, "void set_debug_motor(int status, int motor_id)",
    (void*)change_switch_status, "void change_switch_status(bool status))",
    (void*)clear_pid_param, "void clear_pid_param(void)",
    (void*)gyro_calibration,"void gyro_calibration(void)",
	(void*)accel_calibration,"void accel_calibration(uint8_t where)",
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



















