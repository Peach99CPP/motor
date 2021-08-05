#include "tim_control.h"
int encoder_val[5];//Ĭ��Ϊ0
short status_flag[5];//
int rising_val[5],falling_val[5];
int direct_[5];
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

#define  forward    1
#define  backward   -1
#define speed_param    10000
/*
*@name:HAL_TIM_IC_CaptureCallback
*@function:����������ݣ�������
*@param:��ʱ���ṹ��
*@return:��
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    
    if(htim->Instance == TIM3)//�ж϶�ʱ��
    {
        if(htim->Channel == TIM_CHANNEL_1)//�жϴ�����ͨ�� 
        {
           if(!status_flag[1])//��ʼ����״̬��־λ�ж�,��ʱ����������
           {
              status_flag[1] = 1;//��־�Ѿ�����������
              rising_val[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//��ȡ��ʱ��CNTֵ
               /*
                *�˴������жϵ����ת��
                *�ж�AB ��λ�����λ�ù�ϵ���ó���ת��
               */
              if(HAL_GPIO_ReadPin(MOTOR1_ENCODER_GPIO_Port,MOTOR1_ENCODER_Pin) == GPIO_PIN_RESET)
              {
                  direct_[1] = forward;//ǰ������ת
              }
              else
              {
                  direct_[1] = backward;//���ˣ���ת
              }
              //�Ѳ���������Ϊ�½��ز��񡣵ȴ��½��صĵ���
              __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
           }
           else//�����½���
           {
                status_flag[1] = 0;//��ձ�־λ
                falling_val[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//��ȡ��ʱ��CNTֵ
                encoder_val[1]=(speed_param/(falling_val[1]-rising_val[1]))*direct_[1];//������ֵ������ת����ת�٣����Ϸ���
               __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//���������ز��񣬻ص���һ��
           }
        }
        else if(htim->Channel == TIM_CHANNEL_3)//MOTOR2
        {
           if(!status_flag[2])
           {
              status_flag[2] = 1;
              rising_val[2] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
              if(HAL_GPIO_ReadPin(MOTOR2_ENCODER_GPIO_Port,MOTOR2_ENCODER_Pin) == GPIO_PIN_RESET)
              {
                  direct_[2] = forward;//
              }
              else
              {
                  direct_[2] = backward;
              }
              __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);
           }
           else
           {
                status_flag[2] = 0;
                falling_val[2] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
                encoder_val[2]=(speed_param/(falling_val[2]-rising_val[2]))*direct_[2];
               __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);
           }
        }
    }
    else if( htim->Instance == TIM5)
    {
        if(htim->Channel==TIM_CHANNEL_1)//MOTOR3
        {
            if(!status_flag[3])
            {
                status_flag[3]=1;
                rising_val[3]=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);
                if(HAL_GPIO_ReadPin(MOTOR3_ENCODER_GPIO_Port,MOTOR3_ENCODER_Pin) == GPIO_PIN_RESET)
                {
                    direct_[3] = forward;
                }
                else{
                    direct_[3]=backward;
                }
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
            }
            else
            {
                status_flag[3]=0;
                falling_val[3]=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);
                encoder_val[3]=(speed_param/(falling_val[3]-rising_val[3]))*direct_[3];
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);
            }
        }
        
    }
    else if(htim->Instance  == TIM1)
    {
        if(htim->Channel == TIM_CHANNEL_3)//MOTOR4
        {
            if(!status_flag[4])
            {
                status_flag[4]=1;
                rising_val[4]=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
                if(HAL_GPIO_ReadPin(MOTOR4_ENCODER_GPIO_Port,MOTOR4_ENCODER_Pin) == GPIO_PIN_RESET)
                {
                    direct_[4]=forward;
                }                    
                else
                {
                    direct_[4]=backward;
                }
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);
            }
            else
            {
                status_flag[4]=0;
                falling_val[4]=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
                encoder_val[4]=(speed_param/(falling_val[4]-rising_val[4]))*direct_[4];
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);
            }
        }
    }
}