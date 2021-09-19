# �й������˴����Զ��ּ��Զ��ּ� ==��̱���==
**���α�����Ҫ��Ϊ5�󲿷�**
1. *[�������](#����)*  
   - [�ŵ�](#�ŵ�)
   - [Դ��](#���Դ��)
   - [Ч��](#Ч��ͼƬ)

2. *[�ײ�������](#���)*   
   - [���õ��·���](#���������)
   - [���⼰�������](#����������)
   - [����Ч��](#������Ч��չʾ)
   - [�Ż�����](#�������Ż�����)
4. *[������]*
5. *[ѭ���沿��]*
6. *[��е�۲���]*
7. *[����ϵͳ�����߼�]*
- - -
<span id="����"></span>
## �������
#### ����
<span id="�ŵ�"></span>�������һ��������ԭ�еĻ����ϼ�����==USMART==�������, ��������PID����ʱ�����ش�����ã�������������ظ���¼�ķ����Լ����ڴ�������Ӱ�죬�����˹���Ч�ʡ���ƽʱ���и������ʱ��������䡣
<span id="���Դ��"></span>
#### ***�û�����USMART��Դ��***
```C
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
    (void*)Wait_Switches, "void Wait_Switches(int dir)",
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

```
<span id="Ч��ͼƬ"></span>
#### ����������ʾЧ��
�˴�Ӧ��һ��ͼƬ
- - -
<span id="���"></span>
## �ײ����������֣����ģ�
<span id="���������"></span>
- #### �����ŵ�
  ���ڽ����������һ�������ذ�������ƽ׶ξ;��������������Ķ�ʱ������ʽ��������������������ʵʱת�٣�<span id="ʵ��"></span>���ǶԱ������������ڽ��м�Ⲣ��������ӳ��ת�����������ʱ����Ϊת�ٵļ����׼����������ļ��㷽�����صļ�����ʽ�������Բ�ѯ���·�����������Ϊ��ѯƵ�ʵı仯����ֵ�仯����������ݵ�׼ȷ�ȡ�
  <span id="����������"></span>
- #### ʵ�ֹ����з��ֵ�**����**����**�������**
    1. ���ڲ����Ե�ѡ��
       ���ڲ����Ե�ѡ��һ��ʼѡ�����˫���ؼ�ⷽ��������δ���Ե����ط����������ȱ�㡣�������������ʱ��������==��ת��==����£����õ����ؼ�⽫����Ϊ���ڹ������޷���⣬����**�����ڵȴ���һ�����ڱ��ص����Ĺ������޷��˳�**����Ȼ�����صļ���߼���ʵ�ֽ�Ϊ�򵥣����޷������ת�ٵ����⡣�������õ���==˫����==��ⷽ����
    2. �������ݴ�������
       ���ڷ����ź�ë�̵Ĵ��ڣ������ת��ֵԭʼֵ���ڲ�С�Ĳ��������ֱ�Ӳ���ԭʼ���ݣ����ή���ȶ��ԣ��ر�����Ϊë�̸����˶�ת��������жϣ�����˲��õ�һ������ʵֵ�෴���쳣ֵ�����ڴ����⣬�ھ�����صĲ���֮�󣬾��������˲������Լ����뷴���⣬�����ת������
       �����С�
       **�����Ǵ���ʾ������1�ŵ��Ϊ����**
```C
/**********************************************************************
  * @Name    HAL_TIM_IC_CaptureCallback
  * @declaration : handle tim ic event for encoder
  * @param   htim: [����/��] tim structure ptr
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    double temp_val = 0;
    if(htim == motor1.IC.Tim && htim->Channel == motor1.IC.Active_Channel)
    {
        if(!status_flag[1])//��һ�β����ǲ���������
        {
            status_flag[1] = 1;//״̬��־λ��1���´ν��ж�������һ��
            rising_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//��ȡ��ʱ�����ص�ֵ
            update_count[1] = 0;//�����¼������� ��0
            //�жϷ��򣬷ֱ�����ת���Ƿ�ת
            if(HAL_GPIO_ReadPin(motor1.Encoder_IO.Port, motor1.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[1] = FORWARD;
            }
            else
            {
                direct_[1] = BACKWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_FALLING);//��һ���ǲ����½���
        }
        else//�����½���
        {
            status_flag[1] = 0 ;//״̬λ�����һ������ѭ����ɡ���һ�ξ��ǲ���������
            falling_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//��ȡ�½��ص�ֵ
            cap_temp_val[1] += (SPEED_PARAM / (falling_val[1] - rising_val[1] + TIM_COUNT_VAL * update_count[1])) * direct_[1];//���㱾�εõ���������ӳ��ת�ٵĿ��������ۼ�
            cap_cnt[1]++;//���������ۼӣ����ݲ����������˲��������ֵ

            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING);//������ѭ����ɣ��ص���ʼ״̬��׼���������ؽ��в���

            if(cap_cnt[1] == FILTER)//��������������
            {
                if(!first_flag[1])//��һ�ε�ʱ����Ϊû����һ�ε�ֵ����Ҫ�������⴦��
                {
                    first_flag[1] = 1;
                    encoder_val[1] = cap_temp_val[1] / FILTER;
                }
                else
                {
                    //�ձ�����
                    temp_val = cap_temp_val[1] / FILTER;//��ȡ�����������ڵ�ƽ��ֵ
                    if(!(fabs(temp_val + encoder_val[1]) < THRESHOLD_)) //û����Ϊë�̷����������䣬�еĻ�ֱ���������λ�õ�ֵ
                    {
                        //��ֵ�˲�
                        temp_val += encoder_val[1];
                        encoder_val[1] = temp_val / (2.0);
                        /*�������淽��ԭ������\
                        �и����ڵ�һ�ڶ���ִ�м�϶ʱ���ڣ�������ֵ����ȡ
                        ��ʱ��������ֵ��������ʱֵ���쳣�Ĵ�
                        ���������쳣
                        ��������
                        encoder_val[1] += temp_val;
                        encoder_val[1] /= 2.0;//��ֵ�˲�

                        */
                    }
                }
                //��ر�����0 ���ǵ���0��
                temp_val = 0;
                cap_cnt[1] = 0;
                cap_temp_val[1] = 0 ;
            }

        }


    }
```
3. #### ת��ֵ�Ļ�������
   ��[������ʵ���߼�](#ʵ��)���Ѿ�˵����ͨ����ⷽ���������������ĺ������÷�������һ���������������ڣ���**���ת��Ϊ0ʱ����Ϊ�������ٲ����������ٽ������ת�ٵĺ�����**��Ҳ����˵��==ת��û�еõ����� ͣ������һ��ֵ==,���ڴ����⣬�Ⱥ�������������� ��

   	1. ��ÿһ�ζ�ȡ��ת��ֵ������0
   		�˷��������֮ǰ��ȡ�������Ĳ�������ȡ֮��㽫����0��������Ϊ�˲��Ĳ������˷������ή����ֵ�Ĳ���ʵ�Լ�������ֵ��ʵʱ��Ӧ�ٶȵ͡�
   		
   	2. ��� �Ƿ�ʱ
   		��Ϊ�ڱ�������������л᲻�ϴ��ڶ�ʱ���жϵĲ�������ĳЩ����ʱ�̣�������ֵ�ļ��㻹��������ĸ���Ƶ���йء�����ڶ�ʱ���ж��м����һ�ν�������жϵ�ʱ�䣬���ʱ������һ����ֵ(˵����ʱͣת�����򽫵����ת����0�����ҽ�������ر������³�ʼ��һ�顣�������ԣ��˷����нϸߵĿ����ԣ��ܹ���Ϊ��ʱ׼ȷ�ض�ͣת�����������Ӧ��
**����ʾ��**
```C
/*******************
*@name:HAL_TIM_PeriodElapsedCallback
*@function:���ö�ʱ����ˢ������,����ʱ����ֻ�в��޸�IRQHandler���ܴ����˺���
*@param:��ʱ���ṹ��
*@return:��
**********************/
void MY_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        //���ڼ���������������;������ʱ�������¼������
        if(++update_count[1] >= 3) //�������ж��¼�����̫�࣬˵����ʱ�ĵ�����ڲ�ת��״̬���ʵ��ת����0
        {
            cap_cnt[1] = 0;//�������� �˲���
            cap_temp_val[1] = 0 ;//������ʱת�ٴ洢ֵ
            status_flag[1] = 0;//�ص��������صĲ���
            update_count[1] = 0;//���ʱ�������
            encoder_val[1] = 0;//ת����0
        }
        if(++update_count[2] >= 3)
        {
            cap_cnt[2] = 0;
            cap_temp_val[2] = 0 ;
            status_flag[2] = 0;
            update_count[2] = 0;
            encoder_val[2] = 0;
        }
    }
    else if(htim->Instance == TIM5)
    {
        if(++update_count[3] >= 3)
        {
            cap_cnt[3] = 0;
            cap_temp_val[3] = 0 ;
            status_flag[3] = 0;
            update_count[3] = 0;
            encoder_val[3] = 0;
        }
        if(++update_count[4] >= 3)
        {
            cap_cnt[4] = 0;
            cap_temp_val[4] = 0 ;
            status_flag[4] = 0;
            update_count[4] = 0;
            encoder_val[4] = 0;
        }
    }

}

```
<span id="������Ч��չʾ"></span>
- #### Ч��
�˴�Ӧ��һ��ͼ
<span id="�������Ż�����"></span>
- #### ����Ŀ��
  ��ǰ�ı������ļ���ֵ��Ϊʲ10^3��������Ϊ������Ե�ת����������г̶ȣ�����Ϊ�������Ĵ󣬵����������ݵ�Ӱ��Ҳ���Ŵ󣬵�ǰ�����£��ȶ�ת��ʱ������ĵõ���ֵ�Ĳ�����Χ��+- 15��Χ����������Ŀ����������Ż����ݴ���Ĳ��֣���һ��==�������ݵĲ����̶�==ͬʱȷ�����ݵ�==��ʵ��==
- - -
<span id="������"></span>
## ������
#### ѡ��
������ʹ�õ��������ͺ�Ϊ**����ԭ��ATK_IMU601**������ٷ��������ṩ�˽������ݵĺ��������Ըò�����Ҫ�ǻ�������������ʵ�ֹ��ܵĽ���
#### �ӿ�
