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
#include "openmv.h"
#include "servo.h"

#define Height_HW 5
#define RED_TARGET 1
#define BLUE_TARGET 0

osThreadId Read_Swicth_tasHandle;             //任务句柄
void Read_Swicth(void const *argument);       //函数声明
osThreadId Height_UpadteTask;                 //任务句柄
void HeightUpdate_Task(void const *argument); //函数声明

ScanDir_t Height_Mode = Medium_Head;
Height_t Current_Height = MediumHeight;

Game_Color_t Current_Color = Not_Running;

int MV_Servo_Flag = 1;
int read_task_exit = 1, Height_task_exit = 1; //任务退出标志
static short Height_Flag = 0;

short swicth_status[8]; //开关状态，只在内部进行赋值
short HW_Switch[10];    //红外开关的状态
int MIN_ = 60;
int VERTICAL = 5;

#define SWITCH(x) swicth_status[(x)-1] //为了直观判断开关编号
#define HW_SWITCH(X) HW_Switch[(X)-1]

#define Height_SWITCH HW_Switch[4]            //高度的开关
#define Side_SWITCH(X) HW_Switch[(X) + 5 - 1] //侧边红外的安装位置

void Judge_Side(int color_mode, int dir)
{
    if (color_mode == Red_)
    {
        if (dir == 5)
        {
            Height_Mode = Medium_Head;
        }
        if (dir == 6)
        {
            Height_Mode = Low_Head;
        }
    }
    else if (color_mode == Blue_)
    {
        if (dir == 5)
        {
            Height_Mode = Low_Head;
        }
        if (dir == 6)
        {
            Height_Mode = Medium_Head;
        }
    }
}
void Start_HeightUpdate(void)
{
    Height_Flag = 0;
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Current_Height = MediumHeight;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityNormal, 0, 128);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}
void HeightUpdate_Task(void const *argument)
{
    Height_Flag = 0;
    while (!Height_task_exit)
    {
        if (Height_Mode == Low_Head)
        {
            Current_Height = LowestHeight;
            while (!Height_task_exit)
            {
                if (Get_HW_Status(Height_HW) == on && Height_Flag == 0)
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                }
                if (Height_Flag == 1 && !Servo_Running)
                {
                    if (Get_HW_Status(Height_HW) == off)
                    {
                        Current_Height = MediumHeight;
                    }
                }
                osDelay(20);
            }
        }
        else if (Height_Mode == Medium_Head)
        {
            Current_Height = MediumHeight;
            while (!Height_task_exit)
            {
                if (Get_HW_Status(Height_HW) == on && Height_Flag == 0)
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                }
                if (Height_Flag == 1 && !Servo_Running)
                {
                    if (Get_HW_Status(Height_HW) == off)
                    {
                        Current_Height = LowestHeight;
                    }
                }
                osDelay(20);
            }
        }
    }
    Current_Height = PrimaryHeight;
    vTaskDelete(Height_UpadteTask);
}
void Exit_Height_Upadte(void)
{
    Height_task_exit = 1;
}
/**********************************************************************
  * @Name    Start_Read_Switch
  * @declaration : 启动轻触开关任务
  * @param   None
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Start_Read_Switch(void)
{
    if (read_task_exit)
    {
        read_task_exit = 0; //标记任务开启
        //初始化数组
        memset(swicth_status, 0, sizeof(swicth_status));
        memset(HW_Switch, 0, sizeof(HW_Switch));

        /* definition and creation of Read_Swicth_tas */
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityNormal, 0, 128);
        Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
    }
}

/**********************************************************************
  * @Name    Read_Swicth
  * @declaration : 任务函数实现核心函数
  * @param   argument: [输入/出] 无意义
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Read_Swicth(void const *argument)
{
    while (!read_task_exit)
    {
        //因为GPIO口被配置成pullup，所以只有在低时才是轻触开关导通状态
        if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
            SWITCH(1) = on; //开关状态枚举
        else
            SWITCH(1) = off;

        if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if (HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == GPIO_PIN_RESET)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == GPIO_PIN_RESET)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if (HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin) == GPIO_PIN_RESET)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if (HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin) == GPIO_PIN_RESET)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;
        //红外开关部分
        if (HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin) == GPIO_PIN_RESET)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;

        if (HAL_GPIO_ReadPin(HW_S1_GPIO_Port, HW_S1_Pin) == GPIO_PIN_SET)
            HW_SWITCH(1) = off;
        else
            HW_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_S2_GPIO_Port, HW_S2_Pin) == GPIO_PIN_SET)
            HW_SWITCH(2) = off;
        else
            HW_SWITCH(2) = on;

        if (HAL_GPIO_ReadPin(HW_S3_GPIO_Port, HW_S3_Pin) == GPIO_PIN_SET)
            HW_SWITCH(3) = off;
        else
            HW_SWITCH(3) = on;
        if (HAL_GPIO_ReadPin(HW_S4_GPIO_Port, HW_S4_Pin) == GPIO_PIN_SET)
            HW_SWITCH(4) = off;
        else
            HW_SWITCH(4) = on;
        //定高红外
        if (HAL_GPIO_ReadPin(HW_Height_GPIO_Port, HW_Height_Pin) == GPIO_PIN_SET)
            Height_SWITCH = off;
        else
            //侧边红外
            Height_SWITCH = on;
        if (HAL_GPIO_ReadPin(Side_HW1_GPIO_Port, Side_HW1_Pin) == GPIO_PIN_SET)
            Side_SWITCH(1) = off;
        else
            Side_SWITCH(1) = on;
        if (HAL_GPIO_ReadPin(Side_HW2_GPIO_Port, Side_HW2_Pin) == GPIO_PIN_SET)
            Side_SWITCH(2) = off;
        else
            Side_SWITCH(2) = on;

        osDelay(10); //对请求的频率不高,所以可以10ms来单次刷新
    }
    memset(swicth_status, err, sizeof(swicth_status)); //清空到未初始状态，用于标记此时任务未运行
    vTaskDelete(Read_Swicth_tasHandle);                //从任务列表中移除该任务
    Read_Swicth_tasHandle = NULL;                      //句柄置空
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
    if (read_task_exit)
        return err;    //确保当前任务处在进行中
    return SWITCH(id); //返回对应开关的状态
}

/**********************************************************************
  * @Name    Get_HW_Status
  * @declaration :获取指定ID号红外开关的状态
  * @param   id: [输入/出] 红外开关编号
  * @retval   : 状态
  * @author  peach99CPP
***********************************************************************/
int Get_HW_Status(int id)
{
    if (read_task_exit || (id < 1 || id > 8)) //输入值限制避免出错
        return err;
    return HW_SWITCH(id);
}

/**********************************************************************
  * @Name    Get_Side_Switch
  * @declaration :获取侧边边界开关的状态
  * @param   id: [输入/出] 开关编号
  * @retval   : 状态  扫到了就为on
  * @author  peach99CPP
***********************************************************************/
int Get_Side_Switch(int id)
{
    if (id < 1 || id > 2)
        return off;
    return Side_SWITCH(id);
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
    read_task_exit = 1; //此变量为1 将使得任务函数循环条件不满足
}
/**********************************************************************
  * @Name    Get_Height
  * @declaration : 获取此时应该调用的动作组编号
  * @param   None
  * @retval   : 动作组编号
  * @author  peach99CPP
***********************************************************************/
int Get_Height(void)
{
    return Current_Height;
}

/**********************************************************************
  * @Name    MV_HW_Scan
  * @declaration :利用MV 舵控 联动对阶梯平台进行扫描并抓取
  * @param   dir: [输入/出]  移动的方向
                 1  为车头朝前   向左扫描
                 2  为车头朝前   向右扫描
                 5  为侧身贴边   向前移动
                 6  为侧身贴边   向后移动
**			 enable_imu: [输入/出] 是否使能IMU来保持车身角度的为直线
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void MV_HW_Scan(int r_b, int dir, int enable_imu)
{
    int time_delay = 0;
    mv_rec_flag = 0;
    Action_Gruop(11, 1);                                    //展开爪子
    while (Get_Servo_Flag() == false && time_delay <= 5000) //等待完成，同时避免超时
    {
        time_delay += 10;
        osDelay(10);
    }
    Set_IMUStatus(enable_imu); //设置陀螺仪状态
    Judge_Side(r_b, dir);      //裁决高度判断模式
    // mv_rec_flag = 1;           //避免在最边缘有球而导致卡死，手动设置一下
    Start_HeightUpdate();
    /*1 2为正前方， 5 6为右侧*/
    if (dir == 1)
    {
    dir1_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on) //当未收到MV停止信号或红外开持续导通时
        {
            set_speed(-MIN_, VERTICAL, 0); //一边走一边贴边
            osDelay(10);
        }
        set_speed(0, 0, 0);            //停车
        Wait_Switches(1);              //贴边
        if (Get_HW_Status(dir) == off) //判断退出上述循环的原因，如果是红外触发的，说明此时到达边界，扫描结束，退出函数
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            //是因为收到MV抓球的信号而停止，等到动作组执行完毕后继续扫描，只单次响应
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1; //打开MV处理的开关，为下一次抓球做准备
            Servo_Running = 0;
            goto dir1_Start_Symbol;
        }
    }
    else if (dir == 2)
    {
        Start_HeightUpdate();
    dir2_Start_Symbol:
        set_speed(MIN_, VERTICAL, 0);
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        Wait_Switches(1);
        if (Get_HW_Status(dir) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {

            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            goto dir2_Start_Symbol;
        }
    }
    else if (dir == 5)
    {
        Start_HeightUpdate();
    dir5_Start_Symbol:
        set_speed(VERTICAL, MIN_, 0);
        while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        if (Get_Side_Switch(1) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            Wait_Switches(3);
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            goto dir5_Start_Symbol;
        }
    }
    else if (dir == 6)
    {
        Start_HeightUpdate();
    dir6_Start_Symbol:
        set_speed(VERTICAL, -MIN_, 0);
        while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on)
            osDelay(10);
        set_speed(0, 0, 0);
        if (Get_Side_Switch(2) == off)
        {
            Exit_Height_Upadte();
            return;
        }
        else
        {
            set_speed(0, 0, 0);
            while (Get_Servo_Flag() == false)
                osDelay(10);
            MV_Servo_Flag = 1;
            Servo_Running = 0;
            goto dir6_Start_Symbol;
        }
    }
}

/**********************************************************************
  * @Name    Get_MV_Servo_Flag
  * @declaration : 状态判断接口 用于判断是否可以执行舵机动作组，避免同时发送重复指令
  * @param   None
  * @retval   : 是否可以运行动作组
  * @author  peach99CPP
***********************************************************************/
int Get_MV_Servo_Flag(void)
{
    if (MV_Servo_Flag)
    {
        MV_Servo_Flag = 0;
        return true;
    }
    else
        return false;
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
    /*关于运行时速度的变量,不宜过高否则不稳定*/
    int Switch_Factor = 30;
    int MIN_SPEED = 50;

    if (read_task_exit)
        Start_Read_Switch();

    Set_IMUStatus(false); //关闭陀螺仪,否则设置w速度无意义

    short flag1, flag2, x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //关于参数的解析，根据方向来判定速度的分配方向
        if (dir == 1) //正Y方向
        {
            w1 = 1, w2 = 2;
            x_pn = 0, y_pn = 1;
        }
        else if (dir == 2) //负X方向
        {
            w1 = 4, w2 = 3;
            x_pn = -1, y_pn = 0;
        }
        else if (dir == 3) //正X方向
        {
            w1 = 5, w2 = 6;
            x_pn = 1, y_pn = 0;
        }
        else if (dir == 4) //负Y方向
        {
            w1 = 8, w2 = 7;
            x_pn = 0, y_pn = -1;
        }
    }
    //开始靠近
Closing:
    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0); //设置一个基础速度，此速度与方向参数有关
    //等待开关都开启
    do
    {
        flag1 = Get_Switch_Status(w1); //获取状态
        flag2 = Get_Switch_Status(w2);
        if (flag1 == err || flag2 == err)
        {
            Start_Read_Switch(); //防止此时任务未启动导致卡死循环
            continue;
        }
        /*下面这一句语句，只在单个开关开启时会有作用*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));
        //任务调度
        osDelay(10);

    } while (flag1 == off || flag2 == off); //只有两个都接通，才退出该循环
    osDelay(500);
    if (flag1 == off || flag2 == off)
    {
        MIN_SPEED /= 2; //更低的速度
        if (ABS(MIN_SPEED) < 5)
            goto switch_exit; //防止卡死在这里
        goto Closing;         //继续回到靠近的程序
    }
switch_exit:
    //    Exit_Swicth_Read(); //用完了就关闭任务
    set_speed(0, 0, 0); //开关
    /*******本来这里应该接一个矫正陀螺仪，但是会降低程序的灵活性，所以不添加。在调用本程序之后，自己操作陀螺仪*******/
}

/**********************************************************************
  * @Name    Single_Switch
  * @declaration :检测单边开关
  * @param   switch_id: [输入/出]  开关号 1-8
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Single_Switch(int switch_id)
{
    //    Set_IMUStatus(false); //直接抵着墙撞击，无需陀螺仪稳定角度
    short x, y; //不同方向的速度因子
    short x_vertical, y_vertical;
    int status;         //存储开关状态的变量
    if (read_task_exit) //确保开关的开启状态
        Start_Read_Switch();
    switch (switch_id) //分配速度方向和垂直板子的速度方向
    {
    case 1:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 2:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 3:
        x = 0, y = 1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 4:
        x = 0, y = -1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 5:
        x = 0, y = 1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 6:
        x = 0, y = -1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 7:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    case 8:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    default:
        x = 0, y = 0;
        x_vertical = 0, y_vertical = 0;
    }
RECLOSE:
    while (Get_Switch_Status(switch_id) != on)
    {
        set_speed(x_vertical * VERTICAL, y_vertical * VERTICAL, 0);
        osDelay(5);
    }
    osDelay(500);
    if (Get_Switch_Status(switch_id) != on)
        goto RECLOSE;
    set_speed(x * MIN_ + x_vertical * VERTICAL, y * MIN_ + y_vertical * VERTICAL, 0); //给一个速度,经测试需要在垂直方向上也给一个速度值避免车身被反弹
    do
    {
        status = Get_Switch_Status(switch_id); //获取状态
        if (status == err)
            Start_Read_Switch(); //防止此时任务退出而卡死在循环里
        osDelay(20);             //任务调度
    } while (status == on);      //直到开关断开，此时说明到达边界
    set_speed(0, 0, 0);          //停车
}

/**********************************************************************
  * @Name    Set_SwitchParam
  * @declaration : 调试所用途的函数接口
  * @param   main: [输入/出] 主要速度，沿着边沿移动的速度
**			 vertical: [输入/出]  垂直与边沿的速度，来确保紧贴的状态
  * @retval   :
  * @author  peach99CPP
***********************************************************************/
void Set_SwitchParam(int main, int vertical)
{
    //调试速度的API
    MIN_ = main;         //沿着板子水平方向的速度
    VERTICAL = vertical; //垂直板子的速度，确保紧贴着。
}

/**********************************************************************
  * @Name    HWSwitch_Move
  * @declaration : 单独使用红外来移动到平台的一侧
  * @param   dir: [输入/出]  贴边移动的方向
**			 enable_imu: [输入/出]  是否使能陀螺仪
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void HWSwitch_Move(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        set_speed(-MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on)
            osDelay(10);
    }
    else if (dir == 2)
    {
        set_speed(MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on)
            osDelay(10);
    }
    else if (dir == 5)
    {
        set_speed(VERTICAL, MIN_, 0);
        while (Get_Side_Switch(1) == on)
            osDelay(10);
    }
    else if (dir == 6)
    {
        set_speed(VERTICAL, -MIN_, 0);
        while (Get_Side_Switch(2) == on)
            osDelay(10);
    }
    set_speed(0, 0, 0);
    osDelay(200);
}

/**********************************************************************
  * @Name    MV_HW
  * @declaration :
  * @param   dir: [输入/出] 使用MV和红外来扫描，已经被MV_HW_Scan函数取代
**			 enable_imu: [输入/出]  是否使能陀螺仪
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_HW(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        set_speed(-MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on && Get_Stop_Signal() == false)
            osDelay(10);
    }
    else if (dir == 2)
    {
        set_speed(MIN_, VERTICAL, 0);
        while (Get_HW_Status(dir) == on && Get_Stop_Signal() == false)
            osDelay(10);
    }
    set_speed(0, 0, 0);
    osDelay(100);
}
