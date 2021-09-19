# 中国机器人大赛自动分拣－自动分拣 ==编程报告==
**本次报告主要分为5大部分**
1. *[调试组件](#调试)*  
   - [优点](#优点)
   - [源码](#相关源码)
   - [效果](#效果图片)

2. *[底层电机驱动](#电机)*   
   - [采用的新方案](#电机的区别)
   - [问题及解决方案](#方案及测试)
   - [运行效果](#编码器效果展示)
   - [优化方向](#编码器优化方向)
4. *[陀螺仪]*
5. *[循迹版部分]*
6. *[机械臂部分]*
7. *[总体系统运行逻辑]*
- - -
<span id="调试"></span>
## 调试组件
#### 提升
<span id="优点"></span>今年的新一代程序在原有的基础上加入了==USMART==调试组件, 其在诸如PID调参时起到了重大的作用，相比于以往的重复烧录的繁琐以及对内存寿命的影响，提升了工作效率。在平时进行各项测试时更加灵活多变。
<span id="相关源码"></span>
#### ***用户配置USMART的源码***
```C
#include "usmart.h"
#include "usmart_str.h"
////////////////////////////用户配置区///////////////////////////////////////////////
//这下面要包含所用到的函数所申明的头文件(用户自己添加)
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

//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS==1 	//如果使能了读写操作
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
//函数控制管理器初始化
//得到各个受控函数的名字
//得到函数总数量
struct _m_usmart_dev usmart_dev =
{
    usmart_nametab,
    usmart_init,
    usmart_cmd_rec,
    usmart_exe,
    usmart_scan,
    sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), //函数数量
    0,	  	//参数数量
    0,	 	//函数ID
    1,		//参数显示类型,0,10进制;1,16进制
    0,		//参数类型.bitx:,0,数字;1,字符串
    0,	  	//每个参数的长度暂存表,需要MAX_PARM个0初始化
    0,		//函数的参数,需要PARM_LEN个0初始化
};

```
<span id="效果图片"></span>
#### 串口助手显示效果
此处应有一张图片
- - -
<span id="电机"></span>
## 底层电机驱动部分（核心）
<span id="电机的区别"></span>
- #### 区别及优点
  由于今年采用了新一代的主控板且在设计阶段就决定不采用以往的定时器集成式编码器方案来计算电机的实时转速，<span id="实现"></span>而是对编码器方波周期进行检测并反比例反映出转速情况，采用时间作为转速的计算标准，相对以往的计算方波边沿的计数方式并周期性查询，新方法不会有因为查询频率的变化而数值变化，提高了数据的准确度。
  <span id="方案及测试"></span>
- #### 实现过程中发现的**问题**及其**解决方案**
    1. 关于捕获极性的选择
       关于捕获极性的选择，一开始选择的是双边沿检测方法，但因未尝试单边沿方法的相对优缺点。后来在驱动电机时，发现在==低转速==情况下，采用单边沿检测将会因为周期过长而无法检测，导致**卡死在等待下一个周期边沿到来的过程中无法退出**。虽然单边沿的检测逻辑与实现较为简单，但无法解决低转速的问题。故最后采用的是==双边沿==检测方法。
    2. 关于数据处理问题
       由于方波信号毛刺的存在，电机的转速值原始值存在不小的波动，如果直接采用原始数据，将会降低稳定性，特别是因为毛刺干扰了对转动反向的判断，将会瞬间得到一个与真实值相反的异常值，对于此问题，在经过相关的测试之后，决定进行滤波处理以及加入反向检测，避免对转动方向
       的误判。  
**以下是代码示例，以1号电机为例。**
```C
/**********************************************************************
  * @Name    HAL_TIM_IC_CaptureCallback
  * @declaration : handle tim ic event for encoder
  * @param   htim: [输入/出] tim structure ptr
  * @retval   : void
  * @author  peach99CPP
***********************************************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    double temp_val = 0;
    if(htim == motor1.IC.Tim && htim->Channel == motor1.IC.Active_Channel)
    {
        if(!status_flag[1])//第一次捕获是捕获到上升沿
        {
            status_flag[1] = 1;//状态标志位置1，下次进中断是在下一步
            rising_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//读取此时上升沿的值
            update_count[1] = 0;//更新事件计数器 置0
            //判断方向，分辨是正转还是反转
            if(HAL_GPIO_ReadPin(motor1.Encoder_IO.Port, motor1.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[1] = FORWARD;
            }
            else
            {
                direct_[1] = BACKWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_FALLING);//下一次是捕获下降沿
        }
        else//捕获到下降沿
        {
            status_flag[1] = 0 ;//状态位清除，一个捕获循环完成。下一次就是捕获上升沿
            falling_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);//读取下降沿的值
            cap_temp_val[1] += (SPEED_PARAM / (falling_val[1] - rising_val[1] + TIM_COUNT_VAL * update_count[1])) * direct_[1];//计算本次得到的脉宽。反映出转速的快慢，并累加
            cap_cnt[1]++;//采样次数累加，根据采样次数和滤波次数求均值

            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING);//本采样循环完成，回到初始状态，准备对上升沿进行采样

            if(cap_cnt[1] == FILTER)//采样次数到达了
            {
                if(!first_flag[1])//第一次的时候，因为没有上一次的值，需要进行特殊处理
                {
                    first_flag[1] = 1;
                    encoder_val[1] = cap_temp_val[1] / FILTER;
                }
                else
                {
                    //普遍的情况
                    temp_val = cap_temp_val[1] / FILTER;//获取本采样周期内的平均值
                    if(!(fabs(temp_val + encoder_val[1]) < THRESHOLD_)) //没有因为毛刺发生方向跳变，有的话直接舍弃本次获得的值
                    {
                        //均值滤波
                        temp_val += encoder_val[1];
                        encoder_val[1] = temp_val / (2.0);
                        /*舍弃下面方法原因在于\
                        有概率在第一第二句执行间隙时间内，编码器值被读取
                        此时编码器的值加上了临时值会异常的大
                        容易引起异常
                        故舍弃。
                        encoder_val[1] += temp_val;
                        encoder_val[1] /= 2.0;//均值滤波

                        */
                    }
                }
                //相关变量清0 ！记得清0！
                temp_val = 0;
                cap_cnt[1] = 0;
                cap_temp_val[1] = 0 ;
            }

        }


    }
```
3. #### 转速值的回零问题
   在[编码器实现逻辑](#实现)中已经说过，通过检测方波边沿来进入计算的函数。该方法存在一个显著的问题在于，当**电机转速为0时，因为方波不再产生，将不再进入计算转速的函数中**，也就是说，==转速没有得到更新 停留在上一个值==,对于此问题，先后有两个解决方案 ：

   	1. 在每一次读取完转速值后将其清0
   		此方案借鉴了之前读取编码器的操作，读取之后便将其清0，但是因为滤波的操作，此方法将会降低数值的真实性以及导致速值的实时反应速度低。
   		
   	2. 检测 是否超时
   		因为在编码器计算过程中会不断存在定时器中断的产生，在某些特殊时刻，编码器值的计算还与编码器的更新频率有关。因此在定时器中断中检测上一次进入边沿中断的时间，如果时长超过一定阈值(说明此时停转），则将电机的转速清0，并且将其他相关变量重新初始化一遍。经过测试，此方法有较高的可行性，能够较为及时准确地对停转的情况做出反应。
**代码示例**
```C
/*******************
*@name:HAL_TIM_PeriodElapsedCallback
*@function:利用定时器来刷新任务,计算时长，只有不修改IRQHandler才能触发此函数
*@param:定时器结构体
*@return:无
**********************/
void MY_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        //用于计算脉宽，处理捕获中途发生定时器更新事件的情况
        if(++update_count[1] >= 3) //当更新中断事件发生太多，说明此时的电机处于不转的状态，故电机转速置0
        {
            cap_cnt[1] = 0;//重新启动 滤波器
            cap_temp_val[1] = 0 ;//重置临时转速存储值
            status_flag[1] = 0;//回到对上升沿的捕获
            update_count[1] = 0;//清空时间计数器
            encoder_val[1] = 0;//转速清0
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
<span id="编码器效果展示"></span>
- #### 效果
此处应有一张图
<span id="编码器优化方向"></span>
- #### 升级目标
  当前的编码器的计算值都为什10^3数量级（为了增大对低转速情况的敏感程度），因为数量级的大，导致误差对数据的影响也被放大，当前方案下，稳定转动时，计算的得到的值的波动范围在+- 15范围，接下来的目标在于如何优化数据处理的部分，进一步==降低数据的波动程度==同时确保数据的==真实性==
- - -
<span id="陀螺仪"></span>
## 陀螺仪
### 选型
本程序使用的陀螺仪型号为**正点原子ATK_IMU601**，因其官方例程中提供了解析数据的函数，所以该部分主要是基于陀螺仪数据实现功能的讲解   
#### 结构体介绍:
<u>当运行环境发生改变时，只需要修改结构体成员变量的值即可</u>
```c

ATK_IMU_t  imu =
{
    /*移植时只需要修改以下结构体变量即可*/

    .imu_uart = &huart6,             //串口号
    .yaw_ptr = &(attitude.yaw),     //解析出来的原始数据的指针
    .target_angle = 0,              //pid的目标角度
    .init_angle = 0,                //初始化角度，补偿上电时的初始角度
    .switch_ = 1,                   //使能开关
    .get_angle = Get_Yaw             //函数指针，返回经过限幅和相对0的角度
};
```
#### 2.函数接口介绍
   1. 解析函数本质上调用的是正点原子的解析函数。因为在**实际运行环境**下，会发现原有的单字节接收处理方法很容易触发ORE中断而卡死在中断（<u>与循迹板的接收函数同样问题</u>）
因为此时系统内有其他多个中断正在运行，串口收到的字节极易未被CPU处理就直接被下一个字节覆盖而触发ORE中断。为避免出现上述问题，采用DMA进行传输。
**以下为自行编写的接收函数,通过DMA接收数据，然后再对一整帧的数据进行解析**
```c

/**********************************************************************
  * @Name    IMU_IRQ
  * @declaration :陀螺仪的中断处理函数，处理DMA收到的数据,在DMA 开启的情况下，放在HAL_UART_RxCpltCallback中
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/

void IMU_IRQ(void)
{

    for(uint8_t i = 0; i < BUFFER_SIZE; ++i)//开始遍历DMA接收到的数据
    {
        if(imu901_unpack(imu_cmd[i]))//接收完成
            atkpParsing(&rxPacket);//开始解码，得到姿态角，此函数是正点原子例程中的函数
    }

    HAL_UART_Receive_DMA(imu.imu_uart, imu_cmd, BUFFER_SIZE);//再次开启DMA
}
```
2.角度限幅函数，用于将角度转换到+-180范围之内，实现传入参数的规范化
```c

/**********************************************************************
  * @Name    angle_limit
  * @declaration :
  * @param   angle: [输入/出]
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

float  angle_limit(float  angle)
{
    //把传进来的角度限制在正负180范围
limit_label:
    //采用while的原因在于相比IF可以实现更大的处理范围
    while(angle > 180) angle -= 360;
    while(angle <= -180) angle += 360;
    if(ABS(angle) > 180) goto limit_label;//意义不大，但是避免出错
    return angle;
}
```
3.

