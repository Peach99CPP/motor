/* ************************************************************
  *
  * FileName   : read_status.c
  * Version    : v1.0
  * Author     : peach99CPP
  * Date       : 2021-09-12
  * Description:
  ******************************************************************************
 */
#include "read_status.h "

#include "chassis.h"
#include "atk_imu.h"

osThreadId Read_Swicth_tasHandle;//任务句柄
void Read_Swicth(void const * argument);//函数声明



int read_task_exit = 1;//任务退出标志

short swicth_status[8];//开关状态，只在内部进行赋值

#define SWITCH(x) swicth_status[(x)-1] //为了直观判断开关编号




/**********************************************************************
  * @Name    Start_Read_Switch
  * @declaration : 启动轻触开关任务
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Start_Read_Switch(void)
{
    read_task_exit = 0;//标记任务开启
    memset(swicth_status, 0, sizeof(swicth_status));//初始化数组
    /* definition and creation of Read_Swicth_tas */
    osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityNormal, 0, 128);
    Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
}


/**********************************************************************
  * @Name    Read_Swicth
  * @declaration : 任务函数实现核心函数
  * @param   argument: [输入/出] 无意义
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Read_Swicth(void const * argument)
{
    while(!read_task_exit)
    {
        //因为GPIO口被配置成pullup，所以只有在低时才是轻触开关导通状态
        if(HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
            SWITCH(1) = on;//开关状态枚举
        else
            SWITCH(1) = off;

        if(HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if(HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if(HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == GPIO_PIN_RESET)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if(HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == GPIO_PIN_RESET)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if(HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) == GPIO_PIN_RESET)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if(HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) == GPIO_PIN_RESET)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if(HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) == GPIO_PIN_RESET)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;

        osDelay(50);//对请求的频率不高,所以可以50ms来单次刷新
    }
    memset(swicth_status, err, sizeof(swicth_status));//清空到未初始状态，用于标记此时任务未运行
    vTaskDelete(Read_Swicth_tasHandle);//从任务列表中移除该任务
    Read_Swicth_tasHandle = NULL;//句柄置空
}



/**********************************************************************
  * @Name    Get_Switch_Status
  * @declaration : 获取指定ID开关的通断状态
  * @param   id: [输入/出] 电机编号（1-8）
  * @retval   : 该开关的通断状态
  * @author  peach99CPP
***********************************************************************/
int Get_Switch_Status(int id)
{
    if(read_task_exit) return err;//确保当前任务处在进行中
    return SWITCH(id);//返回对应开关的状态
}



/**********************************************************************
  * @Name    Exit_Swicth_Read
  * @declaration : 退出查询开关状态的任务
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Exit_Swicth_Read(void)
{
    read_task_exit = 1;//此变量为1 将使得任务函数循环条件不满足
}



/**********************************************************************
  * @Name    Wait_Switches
  * @declaration :碰撞轻触开关的实现全过程
  * @param   dir: [输入/出]
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Wait_Switches(int dir)
{
    /*关于运行时速度的宏定义,不宜过高否则不稳定*/
#define Switch_Factor 10
#define MIN_SPEED 50

    if(read_task_exit) Start_Read_Switch();

    imu.switch_ = false;

    short flag1, flag2, x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //关于参数的解析，根据方向来判定速度的分配方向
        if(dir == 1) //正Y方向
        {
            w1 = 2, w2 = 1;
            x_pn = 0, y_pn = 1;
        }
        else if(dir == 2 ) //正X方向
        {
            w1 = 3, w2 = 4;
            x_pn = 1, y_pn = 0;
        }
        else if(dir == 3 )//负X方向
        {
            w1 = 5, w2 = 6;
            x_pn = -1, y_pn = 0;

        }
        else if(dir == 4)//负Y方向
        {
            w1 = 7, w2 = 8;
            x_pn = 0, y_pn = -1;
        }
    }

    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0);//设置一个基础速度，此速度与方向参数有关
    //等待开关都开启
    do
    {
        flag1 = Get_Switch_Status(w1);//获取状态
        flag2 = Get_Switch_Status(w2);
        /*下面这一句语句，只在单个开关开启时会有作用*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));

        if(flag1 == err || flag2 == err ) Start_Read_Switch();//防止此时任务未启动导致卡死循环
        //任务调度
        osDelay(50);

    }
    while( flag1 == off || flag2 == off);//只有两个都接通，才退出该循环
    Exit_Swicth_Read();//用完了就关闭任务
    set_speed(0, 0, 0);//开关
    /*******本来这里应该接一个矫正陀螺仪，但是会降低程序的灵活性，所以不添加*******/
}
