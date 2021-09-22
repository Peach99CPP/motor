# 中国机器人大赛自动分拣－自动分拣 `编程报告`
**本次报告主要分为`6`大部分**
1. *[调试组件](#调试)*  
   - [优点](#优点)
   - [源码](#相关源码)
   - [效果](#效果图片)

2. *[底层电机驱动](#电机)*   
   - [采用的新方案](#电机的区别)
   - [问题及解决方案](#方案及测试)
   - [运行效果](#编码器效果展示)
   - [优化方向](#编码器优化方向)  
3. *[陀螺仪](#陀螺仪)*  
    - [概括](#陀螺仪信息)
    - [数据获取](#陀螺仪数据获取)
    - [数据应用](#陀螺仪数据应用)   
4. *[循迹版部分](#循迹板部分)*  
    - [信息](#循迹板概述)
    - [拓展板](#循迹拓展板)
    - [主控板](#循迹主控板)
5. *[上层机械臂](#机械臂)*    
6. *[系统整体](#整体框架)*  
    - [运行流程图](#整体流程图)
    - [FreeRTOS](#RTOS)
    - [对外通讯协议](#模块通讯协议)  
 ### <p align="center">[联系作者](#联系方式)</p>    
    
   
   
    
       
- - -
 
<span id="调试"></span>
## 调试组件
#### 提升
<span id="优点"></span>今年的新一代程序在原有的基础上加入了`USMART`调试组件, 其在诸如PID调参时起到了重大的作用，相比于以往的重复烧录的繁琐以及对内存寿命的影响，提升了工作效率。在平时进行各项测试时更加灵活多变。
<span id="相关源码"></span>
#### 串口接收函数（基于正点原子通讯协议进行修改）
因为部分移动端串口调试设备不支持自动末尾追加`0X0D 0X0A`换行符，所以增加了以`!`为结尾信号的通讯模式。   
**根据运行情况的需要，通过`修改宏定义`来实现不同种类设备的支持！**
```c

/**********************************************************************
  * @Name    USART1_IRQHandler
  * @功能说明 usrat1 handler
  * @param   None
  * @返回值
  * @author  peach99CPP
***********************************************************************/

void U1_IRQHandler(void)
{
    uint8_t rec;
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        rec =  huart1.Instance->RDR;
        if(!(USART_RX_STA & 0x8000))//接收未完成
        {
#ifdef USE_BLE
            if(rec == 0x0a || rec == 0x21)//兼容非电脑设备的串口发送 以！为结尾

            {
                USART_RX_STA |= 0x8000;
                return ;
            }
            if( rec == 0x0d ) return ;
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3fff] = rec;
                USART_RX_STA++;
                if(USART_RX_STA & USART_RX_STA & 0x3fff == MAX_SIZE) USART_RX_STA = 0;
            }
#endif
#ifdef USE_ATK
            if(rec == 0x0d)
            {
                USART_RX_STA |= 0x4000;
                return;
            }
            if( USART_RX_STA & 0x4000 && rec == 0x0a)
            {
                USART_RX_STA |= 0x8000;
                return;
            }
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3fff] = rec;
                USART_RX_STA++;
                if(USART_RX_STA & USART_RX_STA & 0x3fff == MAX_SIZE) USART_RX_STA = 0;
            }

#endif
        }
    }
}
```
#### 串口发送函数
基于队列进行管理，需要对printf函数进行重定向，将要打印的字符放入队列。  
```c
//重定义fputc函数,使用队列来进行printf,此时要另外开一个task循环刷
int fputc(int ch, FILE *f)
{
    if(tx_queue != NULL)
    {
        if(vPortGetIPSR())
        {
            BaseType_t if_higher_woken = pdFALSE;
            while( xQueueSendFromISR(tx_queue, &ch, &if_higher_woken) != pdTRUE); //阻塞式。确保此时已经成功把消息放入队列
            portYIELD_FROM_ISR(if_higher_woken);//判断是否需要进行任务调度
        }
        else
        {
            //此时并不是在中断中被调用，可以直接写入数据
            xQueueSend(tx_queue, &ch, 1);
        }
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    return ch;
}
//串口中断处理函数中的部分
if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
{
__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TXE);
BaseType_t xTaskWokenByReceive = pdFALSE;
//发送队列中有数据需要发送
if (xQueueReceiveFromISR(tx_queue, (void*)&rec, &xTaskWokenByReceive) == pdPASS)
huart1.Instance->TDR = rec;
else
//无数据发送就关闭发送中断
__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
}
//增强健壮性，在中断函数中加入对ORE中断，避免卡死在该中断
if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))//处理ORE错误导致卡死在中断里
{
uint8_t tmp;
tmp = USART1->ISR;
tmp = USART1->RDR;
__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
}
```
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
       关于捕获极性的选择，一开始选择的是双边沿检测方法，但因未尝试单边沿方法的相对优缺点。后来在驱动电机时，发现在`低转速`情况下，采用单边沿检测将会因为周期过长而无法检测，导致**卡死在等待下一个周期边沿到来的过程中无法退出**。虽然单边沿的检测逻辑与实现较为简单，但无法解决低转速的问题。故最后采用的是`双边沿`检测方法。
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
3. 转速值的回零问题
       在[编码器实现逻辑](#实现)中已经说过，通过检测方波边沿来进入计算的函数。该方法存在一个显著的问题在于，当**电机转速为0时，因为方波不再产生，将不再进入计算转速的函数中**，也就是说，`转速没有得到更新 停留在上一个值==,对于此问题，先后有两个解决方案 ：
    
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
  当前的编码器的计算值都为什10^3数量级（为了增大对低转速情况的敏感程度），因为数量级的大，导致误差对数据的影响也被放大，当前方案下，稳定转动时，计算的得到的值的波动范围在+- 15范围，   
- 所以接下来的目标在于如何优化数据处理的部分，进一步`降低数据的波动程度`同时确保数据的`真实性`
- - -
<span id="陀螺仪"></span>
## 陀螺仪
<span id="陀螺仪流程图"></span>
**陀螺仪逻辑流程图**
![陀螺仪](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/%E6%9C%AA%E5%91%BD%E5%90%8D%E8%A1%A8%E5%8D%95%20(2).png)   

### 选型
本程序使用的陀螺仪型号为**正点原子ATK_IMU601**，因其官方例程中提供了解析数据的函数，所以该部分主要是基于陀螺仪数据实现功能的讲解   
#### 结构体介绍:
<span id="陀螺仪信息"></span>
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
<span id="陀螺仪数据获取"></span>
#### 2.1 **陀螺仪数据接口**
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
3.角度初始化函数（参数为想设定当前角度为多少度）  
    <u>因为陀螺仪上电时默认为0度，且不随着软件复位而复位   
故编写此函数，实现将当前角度值设定为指定角度(软件层面)而无需修改IMU硬件</u>  
***核心思想在于设置补偿角，用于补偿上电时的误差角度***
```c

/**********************************************************************
  * @Name    Set_InitYaw
  * @declaration : 设置当前角度为xx度
  * @param   target: [输入/出]  想设置的角度值
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Set_InitYaw(int target)
{
    imu.switch_ = 0;//先把陀螺仪关掉，否则容易在修改过程中异常

    imu.target_angle = angle_limit(target);//同步修改

    float last_yaw, current_yaw;
    int init_times = 0;

    current_yaw = last_yaw = * imu.yaw_ptr;//获取当前原生数据
    while(init_times < INIT_TIMES)//陀螺仪未达到稳定
    {
        if(fabs( current_yaw - last_yaw) < ERROR_THRESHILD)//偏移很小
            init_times ++;
        else init_times = 0;//飘了，清0，重置
        //更新数值
        last_yaw = current_yaw;
        current_yaw = * imu.yaw_ptr;
        //10ms一次查询,阻塞式
        HAL_Delay(10);
    }
    //陀螺仪稳定，开始获取数据
    imu.init_angle = angle_limit(- angle_limit(current_yaw) + angle_limit(target));
    //恢复陀螺仪的使能状态
    imu.switch_ = 1;
}

```
4. 基于上一个函数实现了软件层面的**指定初始角度的修改**，因此使用的不再是传感器返回的数据，而是经过处理的函数  
   此函数用于返回**经过处理的角度**，结构体中的函数指针就是指向此函数
```c

/**********************************************************************
  * @Name    Get_Yaw
  * @declaration :获取经过限幅的相对于上电位置的Yaw角
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/

float Get_Yaw(void)
{
    float  angle = *(imu.yaw_ptr) + imu.init_angle ;//获取当前原生数据加上补偿角
    return angle_limit(angle);
}
```
***上述函数在移植时可以直接使用，只需要根据实际修改结构体的成员变量***
- - -
<span id="陀螺仪数据应用"></span>
#### 2.2 **基于陀螺仪数据的应用**
<u>全部源码在imu_pid.c</u>  
        *核心思想在于设置想转到的角度为PID的目标值，而后利用实时系统来周期性刷新PID函数   
        并将其控制值加在速度上*
1. 转弯的入口函数
```c

/**********************************************************************
  * @Name    turn_angle
  * @declaration : 转弯的函数实现，可实现绝对角度的转弯和相对角度的转弯
  * @param   mode: [输入/出]  转弯的类型
                    relative(1): 相对角度，相对当前位置转动
                    absolute(2): 绝对角度，转动到绝对角度（软件设定的0°）
**			 angle: [输入/出]  角度数值
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void turn_angle(int mode, int angle)
{
    if(imu.switch_)//检查使能状态
    {
        //限幅
        angle = angle_limit(angle);
        //相对角度模式
        if(mode == relative)
            imu.target_angle = angle_limit(imu.get_angle() + angle);
        //绝对角度模式
        else if( mode == absolute )
            imu.target_angle = angle;
        while(!get_turn_status()) osDelay(10);//转动结束再退出该函数
        //开三秒循迹后关闭，防止转弯后车身偏离线太多导致循迹无法修正
        x_leftbar.if_switch  = true;
        x_rightbar.if_switch = true;
        y_bar.if_switch = true;
        osDelay(3000);
        x_leftbar.if_switch  = false;
        x_rightbar.if_switch = false;
        y_bar.if_switch = false;
    }
}
```
2. 陀螺仪修正函数(底层实现逻辑)
依赖此函数的周期调用来实现车身角度的修正
```c

/**********************************************************************
  * @Name    imu_correct_val
  * @declaration : imu pid实现的核心函数
  * @param   None
  * @retval   :
  * @author  peach99CPP
***********************************************************************/

float imu_correct_val(void)
{
    static float now_angle;//过程变量，为避免重复声明，使用静态变量
    //判断此时转弯的状态
    if(fabs(imu.get_angle() - imu.target_angle) < 2.0) if_completed = 1;
    else if_completed = 1 ;

    if(! imu.switch_ ) return 0; //未使能则直接返回0，不做修改
    else
    {
        imu_data.expect = imu.target_angle;//设置好pid的目标
        now_angle = imu.get_angle();//获取角度数值
        //取最优路径
        if(now_angle - imu.target_angle > 180 ) now_angle -= 360;
        if(now_angle - imu.target_angle < -180 ) now_angle += 360;
        //pid传参
        imu_data.feedback = now_angle;
        //获取PID值
        delta = pos_pid_cal(&imu_data, &imu_para);
        return delta;//返回计算值

    }
}

```
- --
<span id="循迹板部分"></span>
## 循迹板（巡线）
<span id="循迹板概述"></span>
1. 循迹板信息  
本程序使用了宏佳电子的八路激光循迹版，以下是其外观:  
   ![电路板](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/1632028810.jpg)  
   ![对地面](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/35da41ec51cda17610018ee712c5e2b.jpg)  
2. 必要的信息：  
**实现逻辑**    
[循迹板流程图直链](https://m.liuchengtu.com/lct//#R26b8d6fa19f4b990bac03dac30e8a959)
   ![循迹版流程图](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/%E6%9C%AA%E5%91%BD%E5%90%8D%E8%A1%A8%E5%8D%95%20(4).png)
   ![循迹版实现逻辑](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/e15ae7b4d6a179b585d8081ca7f1fc0.jpg)
此款循迹板支持多种输出口，包括**串口**、**IO口**、**PWM**、**ADC**等输出方式
但因为其串口输出最高频率为20Hz，不满足要求，所以只能使用<u>最基础的直接读取IO口电平</u>的方法来实现数据的采集  
此外，由于该款循迹板扫到白线后为低电平输出，但在代码逻辑中读到白线是高，所以最后在输出结果时需要按位进行取反操作或者在进行判断时手动取反。   
   1. 两种控制模式：
      ![循迹控制模式](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/59382e6ecf34f48a803706e3dc9adc1.png)    
       **以下为代码实现，重点在<u>获取陀螺仪使能状态之后的两种处理模式</u>  
   将循迹版的PID用于不同用途**
```c

/************************************************************
*@name:chassis_synthetic_control
*@function:底盘的综合控制函数，包含多种控制
*@param:无
*@return:无
**************************************************************/
void chassis_synthetic_control(void)
{
    static float x, y, w, factor;
    static double max_val;
    if (chassis._switch == false ) return; //如果底盘不被使能，则没有后续操作

    if (++time_count == TIME_PARAM)
    {
        time_count = 0;
        y_error = track_pid_cal(&y_bar);
        x_error = (track_pid_cal(&x_rightbar) - track_pid_cal(&x_leftbar)) / 2.0;
    }

    max_val = 0;//对最大值数据进行初始化
    factor = 1;//倍率因子初始化
    min_val = chassis.x_speed;
    if(min_val > y) min_val = chassis.y_speed;
    if(min_val > w) min_val = chassis.w_speed;
    if(min_val > 100)
    {
        speed_factor = min_val / 100.0;
    }
    else speed_factor = 1;

    if(Get_IMUStatus())
    {
        //陀螺仪开启时的巡线模式，已测试通过
        w_error = imu_correct_val();
        x = chassis.x_speed - y_error * speed_factor ;
        y = chassis.y_speed - x_error * speed_factor;
        w = chassis.w_speed + w_error * speed_factor;

        /***************************************
                1*************2
                 *************
                 *************
                 *************
                 *************
                3*************4
        ****************************************/
        motor_target[1] = 0.707 * y + 0.707 * x - Radius_[1] * w;
        motor_target[2] = -0.707 * y + 0.707 * x - Radius_[2] * w;
        motor_target[3] = 0.707 * y - 0.707 * x - Radius_[3] * w;
        motor_target[4] = -0.707 * y - 0.707 * x - Radius_[4] * w ;
    }
    else
    {
        //陀螺仪关闭状态下的巡线，待测试
        x = chassis.x_speed;
        y = chassis.y_speed;
        w = chassis.w_speed;
        /***************************************
                1*************2
                 *************
                 *************
                 *************
                 *************
                3*************4
        ****************************************/
        //明日调试内容
        motor_target[1] = 0.707 * y + 0.707 * x - Radius_[1] * w + speed_factor * (y_error + x_error);
        motor_target[2] = -0.707 * y + 0.707 * x - Radius_[2] * w + speed_factor * (y_error + x_error);
        motor_target[3] = 0.707 * y - 0.707 * x - Radius_[3] * w + speed_factor * (y_error + x_error);
        motor_target[4] = -0.707 * y - 0.707 * x - Radius_[4] * w + speed_factor * (y_error + x_error);
    }

    //再来一个限幅操作，避免单边速度过高导致控制效果不理想
    //



    for (i = 1; i <= 4; ++i) //找出最大值
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    factor = (max_val > MAX_SPEED) ? MAX_SPEED / max_val : 1;
    if (max_val > MAX_SPEED)//最大值是否超限制，进行操作，确保最大值仍在范围内且转速比例 不变
    {
        factor = MAX_SPEED / max_val;
        for (i = 1; i < 4; ++i)
        {
            motor_target[i] *= factor;
        }

    }

    for (i = 1; i <= 4; ++i)
    {
        /*
        *对电机进行遍历
        *首先获取转速期待值
        *读取编码器参数
        *传入PID计算函数得到计算值,以返回值形式传参
        *由计算值控制电机
        */
        motor_data[i].expect = motor_target[i];
        motor_data[i].feedback = read_encoder(i);
        control_val[i] =  pid_control(&motor_data[i], &motor_param);
        if(control_val[i] > MAX_CONTROL_VAL )  control_val[i] = MAX_CONTROL_VAL;
        if(control_val[i] < - MAX_CONTROL_VAL )  control_val[i] = - MAX_CONTROL_VAL;
        set_motor(i, control_val[i]);


    }
//    printf("%.2f  %.2f",motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect);
//    printf("\r\n");

}
```

##### 重要信息：
**由于CubeMX的Bug，默认生成的代码中关于DMA的初始化函数在USART的初始化函数之后  
这将导致IDLE和DMA被失能，所以每一次使用CubeMX生成代码之后需要手动将DMA初始化移动到USART初始化之前！！**    
<span id="循迹拓展板"></span>  
3. 拓展板程序：  
    1. 对IO口电平进行采集,并将采集到的信息进行编码到一个字节中
```c

void track_encode(void)//在定时器中刷新该任务
{
    //预先准备用宏定义参数解析来完成，但有玄学问题，放弃
    /*************1号寻迹板*****************/
    if(HAL_GPIO_ReadPin(T1_1_GPIO_Port, T1_1_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 0;
    else
        track_value[1] &= ~(1 << 0);

    if(HAL_GPIO_ReadPin(T1_2_GPIO_Port, T1_2_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 1;
    else
        track_value[1] &= ~(1 << 1);

    if(HAL_GPIO_ReadPin(T1_3_GPIO_Port, T1_3_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 2;
    else
        track_value[1] &= ~(1 << 2);

    if(HAL_GPIO_ReadPin(T1_4_GPIO_Port, T1_4_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 3;
    else
        track_value[1] &= ~(1 << 3);

    if(HAL_GPIO_ReadPin(T1_5_GPIO_Port, T1_5_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 4;
    else
        track_value[1] &= ~(1 << 4);

    if(HAL_GPIO_ReadPin(T1_6_GPIO_Port, T1_6_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 5;
    else
        track_value[1] &= ~(1 << 5);

    if(HAL_GPIO_ReadPin(T1_7_GPIO_Port, T1_7_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 6;
    else
        track_value[1] &= ~(1 << 6);


    if(HAL_GPIO_ReadPin(T1_8_GPIO_Port, T1_8_Pin) == GPIO_PIN_RESET)
        track_value[1] |= 1 << 7;
    else
        track_value[1] &= ~(1 << 7);



    /*************2号寻迹板*****************/
    if(HAL_GPIO_ReadPin(T2_1_GPIO_Port, T2_1_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 0;
    else track_value[2] &= ~(1 << 0);

    if(HAL_GPIO_ReadPin(T2_2_GPIO_Port, T2_2_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 1;
    else track_value[2] &= ~(1 << 1);

    if(HAL_GPIO_ReadPin(T2_3_GPIO_Port, T2_3_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 2;
    else track_value[2] &= ~(1 << 2);

    if(HAL_GPIO_ReadPin(T2_4_GPIO_Port, T2_4_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 3;
    else track_value[2] &= ~(1 << 3);

    if(HAL_GPIO_ReadPin(T2_5_GPIO_Port, T2_5_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 4;
    else track_value[2] &= ~(1 << 4);

    if(HAL_GPIO_ReadPin(T2_6_GPIO_Port, T2_6_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 5;
    else track_value[2] &= ~(1 << 5);

    if(HAL_GPIO_ReadPin(T2_7_GPIO_Port, T2_7_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 6;
    else track_value[2] &= ~(1 << 6);

    if(HAL_GPIO_ReadPin(T2_8_GPIO_Port, T2_8_Pin) == GPIO_PIN_RESET)
        track_value[2] |= 1 << 7;
    else track_value[2] &= ~(1 << 7);




    /*************3号寻迹板*****************/
    if(HAL_GPIO_ReadPin(T3_1_GPIO_Port, T3_1_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 0;
    else track_value[3] &= ~(1 << 0);

    if(HAL_GPIO_ReadPin(T3_2_GPIO_Port, T3_2_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 1;
    else track_value[3] &= ~(1 << 1);

    if(HAL_GPIO_ReadPin(T3_3_GPIO_Port, T3_3_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 2;
    else track_value[3] &= ~(1 << 2);

    if(HAL_GPIO_ReadPin(T3_4_GPIO_Port, T3_4_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 3;
    else track_value[3] &= ~(1 << 3);

    if(HAL_GPIO_ReadPin(T3_5_GPIO_Port, T3_5_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 4;
    else track_value[3] &= ~(1 << 4);

    if(HAL_GPIO_ReadPin(T3_6_GPIO_Port, T3_6_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 5;
    else track_value[3] &= ~(1 << 5);

    if(HAL_GPIO_ReadPin(T3_7_GPIO_Port, T3_8_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 6;
    else track_value[3] &= ~(1 << 6);


    if(HAL_GPIO_ReadPin(T3_8_GPIO_Port, T3_8_Pin) == GPIO_PIN_RESET)
        track_value[3] |= 1 << 7;
    else 
        track_value[3] &= ~(1 << 7);
}
```
  2. 将状态信息通过约定的协议进行传输
```c
//编码成一个帧
void cmd_encode(uint8_t id, uint16_t sum)
{
    uart_cmd[0] = HEAD_BYTE;
    uart_cmd[1] = id;
    uart_cmd[2] = track_value[id];
    uart_cmd[3] = (sum&0xFF);
    uart_cmd[4] = TAIL_BYTE;
}
//通过串口发送给主控板，为提高实时性，发送频率为200Hz
void send_value(void)
{
    for(uint8_t i=1;i<=3;++i)
    {
    cmd_encode(i,(i+track_value[i]));
    HAL_UART_Transmit(track_uart.uart,uart_cmd,BUFFER_SIZE,0xff);
    delay_ms(5);
    }
}
```
- --
<span id="循迹主控板"></span>
4. 主控板程序
   - 数据接收程序  
   主控板对于寻迹板接收程序的处理，如果采用单字节结接收处理，很可能因为处理不及时而产生与陀螺仪数据相似的ORE中断  
   因此，其也需要使用DMA进行数据的传输工作。  
    以下是循迹板的初始化和DMA的中断处理函数
```c
/**********************************************************************
* @Name    track_bar_init
* @declaration : 对寻迹板需要的相关变量进行初始化的设置
* @param   None
* @retval   : 无
* @author  peach99CPP
  ***********************************************************************/
  void track_bar_init(void)//相关的初始化函数
  {
  dma_count = 0;
  dma_trans_pos = 0;
  //变量参数的初始化
  y_bar.id = forward_bar;//标注身份
  y_bar.line_num = 0;
  y_bar.num = 0;
  y_bar.line_num = 0;
  y_bar.data.expect = 0;//循迹的目标值恒为0
  y_bar.if_switch = true;//使能

  x_leftbar.id = left_bar;
  x_leftbar.line_num = 0;
  x_leftbar.num = 0;
  x_leftbar.line_num = 0;
  x_leftbar.data.expect = 0;
  x_leftbar.if_switch = true;

  x_rightbar.id = right_bar;
  x_rightbar.line_num = 0;
  x_rightbar.num = 0;
  x_rightbar.line_num = 0;
  x_rightbar.data.expect = 0;
  x_rightbar.if_switch = true;
  //开启DMA接收，此处应确保main中DMA初始化函数在串口初始化前运行
  HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma, BUFF_SIZE);
  }
  /*DMA中断处理，采用的是多个缓冲区，避免数据未处理便被直接覆盖，但也因此需要处理缓冲区位置的选择问题*/
  /**********************************************************************
  * @Name    track_IT_handle
  * @declaration : 接收完成中断
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
  void track_IT_handle(void)
  {
  uint8_t pos = get_avaiable_pos();//通过获取这个检查是否有被填充进去
      if(pos != 0xff)
      {
      dma_count++; //大于0代表有未处理的DMA数据
      dma_trans_pos = ((dma_trans_pos + 1) >= MAX_LINE) ? 0 : dma_trans_pos + 1; //此接收器接收完成，。换到空闲接收区进行接收，防止数组下标越界
      HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);//开启DMA接收
      }
      //否则就在原缓冲区继续接收
      else
      {
      memset(memset(track_dma, 0, sizeof(track_dma)), 0, sizeof(memset(track_dma, 0, sizeof(track_dma)))); //清空掉，作用不大，习惯为止
      HAL_UART_Receive_DMA(&TRACK_UART, (uint8_t*)track_dma[dma_trans_pos], BUFF_SIZE);
      }
  }
```
   - 数据处理程序   
    此部分是对循迹版信息的解码处理环节。功能实现的核心依赖于此函数。  
通过对循迹版信息的处理获取当前循迹板的状态，并以此实现数线、巡线等功能。
```c

/**********************************************************************
  * @Name    track_decode
  * @declaration :对DMA收到的数据进行解码计算，实现循迹各项功能的核心代码
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void track_decode(void)
{
    /***相关宏定义****/
#define EDGE_THRESHOLD 3 //在边缘数线模式下，几颗灯亮起时为有效计数
#define NUM_THRESHOLD 6  //非边缘线计算下，判断到达线的数量
#define MIN_NUM 3        //在压线之后，过线了才会计算一根线，根据灯的数量进行计数
#define EDGE_VAL 7      //边缘数线状态下的循迹读回来的值
    
    times_counts++;//总的处理次数，查看此 数据可以判断是否卡DMA
    dma_count--;//待处理数-1
    uint8_t pos = get_avaiable_pos();//获取可用index
    uint8_t led_num = 0;
    float track_value = 0, temp_val; //相关的变量声明
    
    //下面的和检验看情况开启
    if((uint8_t)( track_dma[pos][1] + track_dma[pos][2] ) == track_dma[pos][3]) //和校验
    {
        for(uint8_t i = 0; i < 8; ++i)
        {
            temp_val = (bool)(((track_dma[pos][2] << i) & 0x80)) * track_weight[i];//根据灯亮与否及其权重得到反馈值
            if( temp_val != 0) led_num++;//计算亮的灯数量
            track_value += temp_val;
        }
        switch (track_dma[pos][1])//判断寻迹板ID
        {
        case 1:
            y_bar.data.feedback = track_value;//赋值
            y_bar.num = led_num;//得到灯的数量
            if(y_bar.num  >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
            {
                /*有两种情况，
                *一是当跑在非边缘时，此时灯的数量比较多
                *二是在边缘跑时，此时灯数量较少且需要加多重判断
                */
                y_bar.line_flag  = 1;//此时到线上
            }
            if(edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL ) //边缘数线的情况下，特殊处理
                y_bar.data.feedback = 0;//要放在对线的判断之后；置0是为了防止此时发生偏移
            if(y_bar.line_flag && y_bar.num <= MIN_NUM )
            {
                //使用此机制为了避免因停留在线上而导致线的数量一直重复计数
                y_bar.line_flag = 0;//数线完成
                y_bar.line_num ++;//线数目加一
            }
            break;
        case 2:
            x_leftbar.data.feedback = track_value;
            x_leftbar.num = led_num;

            if(x_leftbar.num >= NUM_THRESHOLD || (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL))
            {
                x_leftbar.line_flag  = 1; //标记到了线上
            }
            if(edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL)
                x_leftbar.data.feedback = 0;
            if(x_leftbar.line_flag && x_leftbar.num <= MIN_NUM ) //避免因为在线上停留而导致的重复计数问题
            {
                x_leftbar.line_flag = 0;
                x_leftbar.line_num ++;
            }
            break;
        case 3:
            x_rightbar.data.feedback = track_value;
            x_rightbar.num = led_num;
            if( x_rightbar.num >= NUM_THRESHOLD || (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL ))
            {
                x_rightbar.line_flag  = 1;
            }
            if(edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL)
                x_leftbar.data.feedback = 0;
            if(x_rightbar.line_flag && x_rightbar.num <= MIN_NUM )
            {
                x_rightbar.line_flag = 0;
                x_rightbar.line_num ++;
            }
            break;
        default://啥也不干
            ;
        }
        memset(track_dma[pos], 0, sizeof(track_dma[pos])); //把用到的清0，等待下次被填充
    }

}
```
  - 数据应用  
    **计算PID值，并将返回值输出到底盘的速度上**
```c
/*
 * 此为循迹版的输出函数
 * PID的目标值已经设置成0
*/
/**********************************************************************
  * @Name    track_pid_cal
  * @declaration : 寻迹板pid计算函数
  * @param   bar: [输入/出] 哪一个寻迹板
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
float track_pid_cal(trackbar_t * bar)
{
    if(bar->if_switch == true)//使能，计算pid值并进行返回
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0;//未使能，不做改变
}

/*
 *下列代码展示了循迹版PID如何参与底盘速度分解
 */
/************************************************************
*@name:chassis_synthetic_control
*@function:底盘的综合控制函数，包含多种控制
*@param:无
*@return:无
**************************************************************/
void chassis_synthetic_control(void)
{
static float x, y, w, factor;
static double max_val;
if (chassis._switch == false ) return; //如果底盘不被使能，则没有后续操作

if (++time_count == TIME_PARAM)
{
time_count = 0;
y_error = track_pid_cal(&y_bar);
x_error = (track_pid_cal(&x_rightbar) - track_pid_cal(&x_leftbar)) / 2.0;
}

max_val = 0;//对最大值数据进行初始化
factor = 1;//倍率因子初始化

w_error = imu_correct_val();
x = chassis.x_speed - y_error ;
y = chassis.y_speed - x_error;
w = chassis.w_speed + w_error;
min_val = x;
if(min_val > y) min_val = y;
if(min_val > w) min_val = w;
```
---

<span id="机械臂"></span>
## 上层机械臂初步方案
- #### 整体描述
  对于机械臂，机械臂的整体驱动依靠于外接的24路舵机板。通过串口发送信息来进行控制。接下来将针对各个任务的具体要求展开描述。
- #### 圆盘机和条形平台
  对于圆盘机和普通平台，为节约时间，在结构设计上我们共设计了拨球和夹取两种方案。在此基础上，针对这两个任务，仅需在巡线过程中，当车头的两个轻触开关闭合后，底盘停止运动，调节机械臂到固定位置后，利用Openmv进行颜色识别和形状识别，然后根据二维码识别得到的颜色，将该颜色的圆球推下至铲子即可，并分流到对应仓库中。
  图一。
- #### 阶梯型平台、码垛台
  初赛：
  在初赛中所需要夹取的物块为圆球和积木块，此外由于条形平台中还含有小球并且由于同时夹取圆球和积木块可能会导致积木块进入圆球仓库，因此我们路线规划为先去拨下条形平台上的小球，然后再去夹取阶梯型平台的圆球，关闭舱门后，最后夹取阶梯型平台的积木块。由于积木块和圆球逗得夹取顺序不同，Openmv需要识别好矩形和圆球的区别并且此处机械臂共需前后识别两次。
  图二。

  复赛和决赛：
  复赛和决赛中所需要夹取的物块为积木块和塑料环，这两次比赛中取消了初赛中的条形平台，阶梯型平台变为两个，并增加了码垛台和圆盘机。因此我们路线规划为先去拨下圆盘机中的小球，然后再去夹取阶梯型平台的积木块和塑料环，由于积木块和塑料环摆放位置的不确定性，因此对于每个位置都需要设置两套动作组。
  图三。
- #### 立桩
  直接调节机械臂进行夹取。稍后完善。
  图四。
- #### 将圆环放置到码垛台上
  直接调节机械臂。稍后完善。
- #### 排下圆球和倒出积木块
  看最终调试效果后再完善。
  

<span id="整体框架"></span>
## 系统整体运行框架
<span id="整体流程图"></span>
- #### 运行框架    
[整体流程图直链](https://m.liuchengtu.com/lct//#R1e79982406d487b28368954fb46414e7)
   ![流程图](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/%E6%9C%AA%E5%91%BD%E5%90%8D%E8%A1%A8%E5%8D%95%20(1).png)
<span id="RTOS"></span>
- #### FreeRTOS
    **在本程序中集成了`FreeRTOS`，使用的是STM32CubeMX中的`CMSIS V1`版本，经过了一定程度的封装**  
 相对优点有： 
  - 提高其移植性，   
  - 同时<u>简化任务创建的操作过程</u>  
  以下是在本程序中用到的任务列表   
    ![任务列表](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/20210920212556.png)  
    每个任务的作用，查阅[运行流程图](#运行流程图)中`任务`部分的介绍。
  <span id="模块通讯协议"></span>
- #### 全局通信协议：
   该协议主要适用于`主控板`与`拓展板`、`openmv`间通讯。  
    查看具体应用：[完整源码](../Module)  
  1.实现逻辑：
  - 发送时的编码  
 <u>以openmv的解码函数举例</u>
      - 帧头
      - 事件ID
      - 正负标志位
      - 参数低8位
      - 参数高8位
      - 校验和
      - 帧尾
 ```c
/**********************************************************************
  * @Name    cmd_encode
  * @declaration : 根据协议编码发送的内容
  * @param   event_id: [输入/出]  时间的类型
**			 param: [输入/出] 参数
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void cmd_encode(const uint8_t event_id, int  param)
{
    static uint8_t pos_flag;
    if(param > 0 ) pos_flag = 1;
    else
    {
        pos_flag = 2;
        param *= -1;
    }

    uint8_t h_byte, l_byte;//获取参数的高8位和低8位
    h_byte = (param >> 8);
    l_byte = (param & 0xff);
    //定义通讯协议
    MV.mv_cmd[0] = START_BYTE;//帧头
    MV.mv_cmd[1] = event_id;//触发的事件id
    MV.mv_cmd[2] = pos_flag;
    MV.mv_cmd[3] = l_byte;//参数高8位
    MV.mv_cmd[4] = h_byte;//参数低8位
    MV.mv_cmd[5] = (uint8_t)(event_id + pos_flag + h_byte + l_byte);//和校验
    MV.mv_cmd[6] = END_BYTE;//帧尾
}
```
  - 接收时解码:   
<u>以openmv的解码函数举例</u>
```c
/**********************************************************************
  * @Name    MV_rec_decode
  * @declaration : 判断接收完成后，对接收的内容进行解码
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_rec_decode(void)
{
    static int pn = 1; //正负标志符
    if(MV.rec_buffer[0] + MV.rec_buffer[1] + MV.rec_buffer[2] + MV.rec_buffer[3] == MV.rec_buffer[4])
    {
        //根据参数内容对参数进行处理
        if( MV.rec_buffer[1] == 1 ) pn = 1;
        else pn = -1;

        mv_param = (MV.rec_buffer[2] +  (MV.rec_buffer[3] << 8)) * pn;
    }
    MV.rec_len = 0;
    MV.RX_Status = 0;
    //处理完之后记得重新初始化结构体中的rec_len和RX_status变量，避免出错
    ;
}
```

  2.待改进的地方：
  - ~~目前的协议在当前的应用范围下未出现问题~~(死要面子 嘴硬)，但对于其他特殊数据，其存在没有解决`透明传输`的问题，当因传输错误或数据内容中出现与帧头帧尾相同的字节时，未有有效的方法来裁决其属于正文信号或是帧尾信号可能因此出现错误。    
目前正计划通过`字符填充`来解决上述问题。**提高鲁棒性**，在参数传输过程中无需考虑其值可能使接收机制产生误判的可能。

  - --
  <span id="联系方式"></span>
# 有问题联系：
- ***13311889904(TEL)***  
       
       
- ***1831427532(QQ)***  
      
       
- ***3120000903@mail2.gdut.edu.cn***     
  ***1831427532@qq.com***     
          
          
- ***Wechat***   
      
   ![联系方式](https://raw.githubusercontent.com/Peach99CPP/pic/main/img/7db7780b6dc421984e3c67cde1c089b.jpg)
    
        
- -- 
