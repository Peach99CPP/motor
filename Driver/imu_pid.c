/* ************************************************************
  *    
  * FileName   : imu_pid.c   
  * Version    : v1.0		
  * Author     : peach99CPP		
  * Date       : 2021-09-11         
  * Description:  ������Ӧ��API
  ******************************************************************************
 */

#include "imu_pid.h"
#include "atk_imu.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

pid_data_t imu_data, anglekeep_data;
pid_paramer_t imu_para =
{
    .kp = 9,
    .ki = 0,
    .kd = 0,
    .integrate_max = 60,
    .control_output_limit = 300
};

extern ATK_IMU_t  imu;
static float delta;




/**********************************************************************
  * @Name    imu_correct_val
  * @declaration :
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

float imu_correct_val(void)
{
    static float now_angle;//���̱�����Ϊ�����ظ�������ʹ�þ�̬����
    if(! imu.switch_ ) return 0; //δʹ����ֱ�ӷ���0�������޸�
    else
    {
        imu_data.expect = imu.target_angle;//���ú�pid��Ŀ��
        now_angle = imu.get_angle();//��ȡ�Ƕ���ֵ
        //ȡ����·��
        if(now_angle - imu.target_angle > 180 ) now_angle -= 360;
        if(now_angle - imu.target_angle < -180 ) now_angle += 360;
        //pid����
        imu_data.feedback = now_angle;
        //��ȡPIDֵ
        delta = pos_pid_cal(&imu_data, &imu_para);
        return delta;//���ؼ���ֵ

    }
}


/**********************************************************************
  * @Name    set_imu_angle
  * @declaration :
  * @param   angle: [����/��] 
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void set_imu_angle(int  angle)
{
    imu.switch_ = 1;//Ĭ�Ͽ�������
    imu.target_angle = angle_limit(angle);//���޷���ĽǶȣ������ṹ��

}

/**********************************************************************
  * @Name    set_imu_param
  * @declaration :
  * @param   p: [����/��] 
**			 i: [����/��] 
**			 d: [����/��] 
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void set_imu_param(int p, int i, int d)
{
    //USMART�������ò���
    imu_para.kp = (p / 10.0);
    imu_para.ki = (i / 100.0);
    imu_para.kd = (d / 10.0);
}
/**********************************************************************
  * @Name    set_imu_status
  * @declaration :
  * @param   status: [����/��] 
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void set_imu_status(int status)
{
    //�ı������ǵ�ʹ��״̬
    imu.switch_ = status;
}



/**********************************************************************
  * @Name    turn_angle
  * @declaration : ת��ĺ���ʵ�֣���ʵ�־��ԽǶȵ�ת�����ԽǶȵ�ת��
  * @param   mode: [����/��]  ת�������
                    relative(1): ��ԽǶ�
                    absolute(2): ���ԽǶ�
**			 angle: [����/��]  �Ƕ���ֵ
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void turn_angle(int mode ,int angle)
{
    if(imu.switch_)
    {
        //�޷�
        angle = angle_limit(angle);
        //��ԽǶ�ģʽ
        if(mode == relative)
            imu.target_angle = angle_limit(imu.get_angle()+ angle);
        //���ԽǶ�ģʽ
        else if( mode == absolute )
            imu.target_angle = angle; 
    }        
    
}
