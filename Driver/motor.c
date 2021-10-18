#include "motor.h"
#include "chassis.h"
#include "cmsis_os.h"
#define MAX_VAL 7000
//#define DEBUG_MODE
int debug_motor_id = 0, switch_status = 0;
int debug_speed = 0;
pid_data_t motor_data[5];
pid_paramer_t motor_param;
//积分限幅放宽，尝试让车更具有动力
float param_[5] = {3500,
                   9900,
                   130,
                   80,
                   10};

motor_t motor1, motor2, motor3, motor4;

/**********************************************************************
  * @Name    motor_init
  * @功能说明 init for motor_t
  * @param   None
  * @返回值  void
  * @author  peach99CPP
***********************************************************************/

void motor_init(void)
{

    /*****************电机1*****************/
    motor1.Encoder_IO.Port = MOTOR1_ENCODER_GPIO_Port; //配置编码器GPIO_PORT
    motor1.Encoder_IO.Pin = MOTOR1_ENCODER_Pin;        //配置PIN

    motor1.IC.Tim = &htim5;                              //确定好编码器的定时器
    motor1.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_1; //这两个通道是同个意思，只不过为了编程时易于区分所以设置不同变量
    motor1.IC.Channel = TIM_CHANNEL_1;                   //

    motor1.PWM.Tim = &htim1;              //PWM驱动的TIM
    motor1.PWM.Channel_A = TIM_CHANNEL_2; //两个PWM通道
    motor1.PWM.Channel_B = TIM_CHANNEL_1; //

    HAL_TIM_PWM_Start(motor1.PWM.Tim, motor1.PWM.Channel_A); //PWM使能
    HAL_TIM_PWM_Start(motor1.PWM.Tim, motor1.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor1.IC.Tim, motor1.IC.Channel); //IC捕获使能
    __HAL_TIM_ENABLE_IT(motor1.IC.Tim, TIM_IT_UPDATE);
    set_motor(1, 0);

    /*****************电机2*****************/
    motor2.Encoder_IO.Port = MOTOR2_ENCODER_GPIO_Port; //配置编码器GPIO_PORT
    motor2.Encoder_IO.Pin = MOTOR2_ENCODER_Pin;        //配置PIN

    motor2.IC.Tim = &htim5;                              //确定好编码器的定时器
    motor2.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_3; //这两个通道是同个意思，只不过为了编程时易于区分所以设置不同变量
    motor2.IC.Channel = TIM_CHANNEL_3;                   //

    motor2.PWM.Tim = &htim1;              //PWM驱动的TIM
    motor2.PWM.Channel_A = TIM_CHANNEL_4; //两个PWM通道
    motor2.PWM.Channel_B = TIM_CHANNEL_3; //

    HAL_TIM_PWM_Start(motor2.PWM.Tim, motor2.PWM.Channel_A); //PWM使能
    HAL_TIM_PWM_Start(motor2.PWM.Tim, motor2.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor2.IC.Tim, motor2.IC.Channel); //IC捕获使能
    __HAL_TIM_ENABLE_IT(motor2.IC.Tim, TIM_IT_UPDATE);
    set_motor(2, 0);

    /*****************电机3*****************/
    motor3.Encoder_IO.Port = MOTOR3_ENCODER_GPIO_Port; //配置编码器GPIO_PORT
    motor3.Encoder_IO.Pin = MOTOR3_ENCODER_Pin;        //配置PIN

    motor3.IC.Tim = &htim3;                              //确定好编码器的定时器
    motor3.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_1; //这两个通道是同个意思，只不过为了编程时易于区分所以设置不同变量
    motor3.IC.Channel = TIM_CHANNEL_1;                   //

    motor3.PWM.Tim = &htim2;              //PWM驱动的TIM
    motor3.PWM.Channel_A = TIM_CHANNEL_3; //两个PWM通道
    motor3.PWM.Channel_B = TIM_CHANNEL_4; //

    HAL_TIM_PWM_Start(motor3.PWM.Tim, motor3.PWM.Channel_A); //PWM使能
    HAL_TIM_PWM_Start(motor3.PWM.Tim, motor3.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor3.IC.Tim, motor3.IC.Channel); //IC捕获使能
    __HAL_TIM_ENABLE_IT(motor3.IC.Tim, TIM_IT_UPDATE);
    set_motor(3, 0);

    /*****************电机4*****************/
    motor4.Encoder_IO.Port = MOTOR4_ENCODER_GPIO_Port; //配置编码器GPIO_PORT
    motor4.Encoder_IO.Pin = MOTOR4_ENCODER_Pin;        //配置PIN

    motor4.IC.Tim = &htim3;                              //确定好编码器的定时器
    motor4.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_3; //这两个通道是同个意思，只不过为了编程时易于区分所以设置不同变量
    motor4.IC.Channel = TIM_CHANNEL_3;                   //

    motor4.PWM.Tim = &htim2;                                 //PWM驱动的TIM
    motor4.PWM.Channel_A = TIM_CHANNEL_2;                    //两个PWM通道
    motor4.PWM.Channel_B = TIM_CHANNEL_1;                    //
    HAL_TIM_PWM_Start(motor4.PWM.Tim, motor4.PWM.Channel_A); //PWM使能
    HAL_TIM_PWM_Start(motor4.PWM.Tim, motor4.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor4.IC.Tim, motor4.IC.Channel); //IC捕获使能
    __HAL_TIM_ENABLE_IT(motor4.IC.Tim, TIM_IT_UPDATE);
    set_motor(4, 0);
}

/**********************************************************************
  * @Name    Motor_PID_Init
  * @功能说明 Init for Motor PID param
  * @param   : [输入/出] void
  * @返回值  void
  * @author  peach99CPP
***********************************************************************/

void Motor_PID_Init()
{
    motor_param.integrate_max = param_[0];
    motor_param.control_output_limit = param_[1];
    motor_param.kp = param_[2];
    motor_param.ki = param_[3];
    motor_param.kd = param_[4];
}

/************************************************************
*@name:read_encoder
*@function:得到编码器的值
*@param:想要读取的电机编号
*@return:编码器的值
**************************************************************/
float read_encoder(int motor_id)
{
    double temp_num = encoder_val[motor_id];
//    encoder_val[motor_id] = 0;
#ifdef DEBUG_MODE
    printf("\r\nmotor%d sppeed =%d\r\n", motor_id, (int)temp_num);
#endif
    //在调试中发现若不清除数据 在断电后转速数据不会得到更新
    return (int)temp_num;
}

/************************************************************
*@name:set_motor
*@function：控制电机的定时器口，驱动电机转动
*@motor_id:控制的目标电机编号
*@control_val：输入的控制值值
*@return: 无
**************************************************************/
void set_motor(int motor_id, int control_val)
{
    volatile uint32_t *ChannelA_ptr;
    volatile uint32_t *ChannelB_ptr;
    int pos_flag; //标志位变量
    //限幅
    if (control_val > MAX_VAL)
    {
        control_val = MAX_VAL;
    }
    else if (control_val < -(MAX_VAL))
    {
        control_val = -MAX_VAL;
    }
    //正负判断
    if (control_val > 0)
    {
        pos_flag = 1;
    }
    else if (control_val < 0 || control_val == 0)
    {
        pos_flag = 0;
        control_val *= (-1); //等价于进行ABS操作
    }

    ChannelA_ptr = get_motor_channelA_ptr(motor_id);
    ChannelB_ptr = get_motor_channelB_ptr(motor_id);
    if (ChannelA_ptr != NULL && ChannelB_ptr != NULL) //避免指针错误操作
    {
        if (pos_flag)
        {
            *ChannelA_ptr = control_val;
            *ChannelB_ptr = 0;
        }
        else
        {
            *ChannelB_ptr = control_val;
            *ChannelA_ptr = 0;
        }
    }
}

/**********************************************************************
  * @Name    get_motor_channelA_ptr
  * @declaration :get the ptr of register of channelA
  * @param   motor_id: [输入/出]
  * @retval   : ptr
  * @author  peach99CPP
***********************************************************************/

volatile uint32_t *get_motor_channelA_ptr(int motor_id)
{
    volatile uint32_t *ptr;
    switch (motor_id)
    {
    case 1:

        if (motor1.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR1);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR2);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR3);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR4);
        }
        break;
    case 2:
        if (motor2.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR1);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR2);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR3);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR4);
        }
        break;
    case 3:
        if (motor3.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR1);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR2);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR3);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR4);
        }
        break;
    case 4:
        if (motor4.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR1);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR2);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR3);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR4);
        }
        break;
    default:
        ptr = NULL;
    }
    return ptr;
}
/**********************************************************************
  * @Name    get_motor_channelB_ptr
  * @declaration :get the ptr of register of channelB
  * @param   motor_id: [输入/出]
  * @retval   : ptr
  * @author  peach99CPP
***********************************************************************/

volatile uint32_t *get_motor_channelB_ptr(int motor_id)
{
    volatile uint32_t *ptr;
    switch (motor_id)
    {
    case 1:

        if (motor1.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR1);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR2);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR3);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR4);
        }
        break;
    case 2:
        if (motor2.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR1);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR2);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR3);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR4);
        }
        break;
    case 3:
        if (motor3.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR1);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR2);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR3);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR4);
        }
        break;
    case 4:
        if (motor4.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR1);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR2);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR3);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR4);
        }
        break;
    default:
        ptr = NULL;
    }
    return ptr;
}

/**********************************************************************
  * @Name    show_speed
  * @declaration :used to show motor speed
  * @param   : [输入/出] none
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void show_speed(void)
{
    if (debug_motor_id == 0 || !switch_status)
        return;
    printf("%.2lf,  %d,  %.2f,  %.2f\r\n", read_encoder(debug_motor_id), (int)motor_target[debug_motor_id], control_val[debug_motor_id], motor_data[debug_motor_id].err);
}

/**********************************************************************
  * @Name    clear_motor_data
  * @declaration : clear pid output for increment pid
  * @param   None
  * @retval   :void
  * @author  peach99CPP
***********************************************************************/

void clear_motor_data(void)
{
    //清除PID积分值,恢复捕获计数值
    for (uint8_t i = 1; i <= 4; ++i)
    {
        pid_clear(&motor_data[i]);
        encoder_val[i] = 0;
        set_motor(i, 0);
        status_flag[i] = 0;
    }
    //重新设置捕获上升沿
    __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_RISING);

    encoder_sum = 0;
}
/******
 * 进行电机调试时所用的函数集合
 * 进行电机调试时,USMART里的宏定义记得设置为1
 * 步骤： 设置好
 * 
 */

/**********************************************************************
  * @Name    set_debug_motor
  * @declaration :set the status and motor id
  * @param   status: [输入/出]
**			 motor_id: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void set_debug_motor(int status, int motor_id)
{

    switch_status = status;
    debug_motor_id = motor_id;
}

/**********************************************************************
  * @Name    motor_debug
  * @declaration : run debug in main function
  * @param   None
  * @retval   :void
  * @author  peach99CPP
***********************************************************************/

void motor_debug(void)
{
    if (debug_motor_id >= 1)//确认是在debug模式下，需要先设置电机编号
    {
        set_speed(0, debug_speed, 0);
        osDelay(1000);
        set_speed(0, 0, 0);
        osDelay(1000);
    }
}

/**********************************************************************
  * @Name    set_debug_speed
  * @declaration :  set debug speed
  * @param   speed: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

void set_debug_speed(int speed)
{
    chassis.enable_switch = true;
    debug_speed = speed;
}
/**********************************************************************
  * @Name    set_motor_pid
  * @功能说明 ki kd kp param changg interface
  * @param   kp: [输入/出] kp param
**			 ki: [输入/出] ki param
**			 kd: [输入/出] kd param
  * @返回值 void
  * @author  peach99CPP
***********************************************************************/

void set_motor_pid(int kp, int ki, int kd)
{
    clear_motor_data();
    motor_param.kp = kp;
    motor_param.ki = ki;
    motor_param.kd = kd;
}

/**********************************************************************
  * @Name    set_motor_maxparam
  * @功能说明 change max value
  * @param   integrate_max: [输入/出]  mav value of I
**			 control_output_limit: [输入/出]  general max value
  * @返回值  void
  * @author  peach99CPP
***********************************************************************/

void set_motor_maxparam(int integrate_max, int control_output_limit)
{
    clear_motor_data();
    motor_param.control_output_limit = control_output_limit;
    motor_param.integrate_max = integrate_max;
}
