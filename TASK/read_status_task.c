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
#include "general.h"
#include "track_bar_receive.h"
#include "Wait_BackInf.h"

#define Height_HW1 5
#define RED_TARGET 1
#define BLUE_TARGET 0
// 17 18 19最低 最高 中间
osThreadId Read_Swicth_tasHandle;             //任务句柄
void Read_Swicth(void const *argument);       //函数声明
osThreadId Height_UpadteTask;                 //任务句柄
void HeightUpdate_Task(void const *argument); //函数声明

ScanDir_t Height_Mode = Primary_Head;
Height_t Current_Height = PrimaryHeight;
Game_Color_t Current_Color = Not_Running;

int Height_id = 1;
int read_task_exit = 1, Height_task_exit = 1; //任务退出标志
short Height_Flag = 0;

bool QR_Brick = false; //是否位于三种高度 模式

short swicth_status[8]; //开关状态，只在内部进行赋值
short HW_Switch[10];    //红外开关的状态
int MIN_ = 40;
int VERTICAL = 10;

bool update_finish = true;
bool HeightAvailable = true;

#define Wait_Servo_Done 20000 //等待动作组完成的最大等待时间

#define SWITCH(x) swicth_status[(x)-1] //为了直观判断开关编号
#define HW_SWITCH(X) HW_Switch[(X)-1]  // 0到3下标就是红外开关的位置

#define Height_SWITCH(x) HW_Switch[(x) + 4 - 1] //高度的开关，第4和第5下标分配给高度红外

#define Side_SWITCH(X) HW_Switch[(X) + 6 - 1] //侧边红外的安装位置,有两个，分配下标为6 和7

bool Get_HeightAvailable(void)
{
    return HeightAvailable;
}
void Set_HeightAvailable(bool Switch_Status)
{
    HeightAvailable = Switch_Status;
}

void Recover_EnableStatus(void)
{
    Set_MV_Mode(true);
    Set_QR_Status(true);
}

/**********************************************************************
 * @Name    Get_Update_Result
 * @declaration :获取高度更新的结果
 * @param   None
 * @retval   : 是否更新完成
 * @author  peach99CPP
 ***********************************************************************/
bool Get_Update_Result(void)
{
    return update_finish;
}

/**********************************************************************
 * @Name    Set_Update_Status
 * @declaration : 设置高度更新值
 * @param   status: [输入/出]  设置后的状态
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_Update_Status(bool status)
{
    update_finish = status;
}

/**********************************************************************
 * @Name    Wait_Update_finish
 * @declaration : 等待更新结束 用这个卡住任务 否则不会停车
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Update_finish(void)
{
    while (Get_Update_Result() != true)
    {
        set_speed(0, 0, 0);
        osDelay(1);
    }
}

/**********************************************************************
 * @Name    Set_NeedUp
 * @declaration : 设置是否为二维码模式
 * @param   if_on: [输入/出] 是或否
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_NeedUp(bool if_on) //在此处设置是否需要动态变换高度
{
    QR_Brick = if_on;
}
/**********************************************************************
 * @Name    Return_IFQR
 * @declaration : 返回是否处于二维码工作模式
 * @param   None
 * @retval   : true则为二维码模式
 * @author  peach99CPP
 ***********************************************************************/
bool Return_IFQR(void)
{
    return QR_Brick;
}

/**********************************************************************
 * @Name    QR_Mode_Height
 * @declaration : 在二维码模式下执行高度变换
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void QR_Mode_Height(void)
{
    if (Return_IFQR())
    {
        Inf_Servo_Height(Get_Height()); //根据高度执行动作组
    }
}
/**********************************************************************
 * @Name    Return_ReverseID
 * @declaration : 获取相对的开关编号
 * @param   id: [输入/出] 当前的开关编号
 * @retval   : 同侧的相对编号
 * @author  peach99CPP
 ***********************************************************************/
int Return_ReverseID(int id)
{
    if (id == 1)
        return 2;
    else if (id == 2)
        return 1;
    else if (id == 5)
        return 6;
    else if (id == 6)
        return 5;
    else
        return 1; //避免出错，返回0容易引起数组越界 todo最好在此分支增加一个错误报告的打印数据
}

void Inf_Servo_Height(int now_height)
{
    if (Return_IFQR()) //在复赛决赛部分需要
    {
        static int last_height = PrimaryHeight;
        if (last_height != now_height) //避免重复提醒
        {
            last_height = now_height;
            printf("高度发生改变\n");
        }
        Set_Update_Status(false);
        set_speed(0, 0, 0);                 //先停车 todo  此处运行之后
        Wait_Servo_Signal(Wait_Servo_Done); //等待上一个命令完成
        if (now_height == LowestHeight)     //根据当前高度进行角度的更新操作
            Action_Gruop(toLowest, 1);      //运行动作组
        else if (now_height == MediumHeight)
            Action_Gruop(toMedium, 1);
        else if (now_height == HighestHeight)
            Action_Gruop(toHighest, 1);
        else
            osDelay(100);                   //初始状态
        Wait_Servo_Signal(Wait_Servo_Done); //等待信号
        Set_Update_Status(true);
    }
    Set_HeightAvailable(true); //在下面的操作中对高度红外的读取进行屏蔽
}
/**********************************************************************
 * @Name    Judge_Side
 * @declaration :
 * @param   color_mode: [输入/出]
 **			 dir: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Judge_Side(int dir)
{
    if (dir == 5)
    {
        Current_Height = MediumHeight;
    }
    else if (dir == 6)
    {
        Current_Height = LowestHeight;
    }
}

/**********************************************************************
 * @Name    Start_HeightUpdate
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Start_HeightUpdate(void)
{
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Height_Flag = 0;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityHigh, 0, 256);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}

/**********************************************************************
 * @Name    HeightUpdate_Task
 * @declaration :
 * @param   argument: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void HeightUpdate_Task(void const *argument)
{
    Height_Flag = 0;
    static short temp_roi_chnage_flag = 1;
    // todo 后续增加参数或者其他赋值的变量来指定使用哪个红外（使用双高度红外的情况下）
    while (!Height_task_exit)
    {
        if (Current_Height == LowestHeight)
        {
            Height_id = 2; //使用2号来判断高度模式 /
            // todo 此处记得检查是否已经设置好对应高度的切换动作组
            printf("\n************Current Height:LowestHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_Servo_Flag() == true && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                    printf("\n************Current Height:HighestHeight***************\n");
                    Inf_Servo_Height(Current_Height);
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        Current_Height = MediumHeight;
                        Height_Flag = 2;
                        printf("\n************Current Height:MediumHeight***************\n");
                        Inf_Servo_Height(Current_Height);
                    }
                }
                osDelay(1);
            }
        }
        else if (Current_Height == MediumHeight)
        {
            Height_id = 1; // todo临时修改
            OpenMV_ChangeRoi(1);
            printf("\n************Current Height:MediumHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Servo_Flag() == true && Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Height_id = 1; // todo 此处需要检查下，因为一开始使用了临时的红外编号
                    Current_Height = HighestHeight;
                    Inf_Servo_Height(Current_Height);
                    printf("\n************Current Height:HighestHeight***************\n");
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        Current_Height = LowestHeight;
                        Height_Flag = 2;
                        Inf_Servo_Height(Current_Height);
                        printf("\n************Current Height:LowestHeight***************\n");
                    }
                    //以下在接收端经不被处理
                    if (Get_Height_Switch(2) == on && temp_roi_chnage_flag == 1)
                    {
                        temp_roi_chnage_flag = 0;
                        OpenMV_ChangeRoi(2);
                    }
                }

                osDelay(1);
            }
        }
        osDelay(5);
    }
    Height_Flag = 0;
    Current_Height = PrimaryHeight;
    vTaskDelete(NULL);
}

/**********************************************************************
 * @Name    Exit_Height_Upadte
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
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
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityAboveNormal, 0, 128);
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
        if (HAL_GPIO_ReadPin(HW_Height1_GPIO_Port, HW_Height1_Pin) == GPIO_PIN_SET)
            Height_SWITCH(1) = off;
        else
            //侧边红外
            Height_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_Height2_GPIO_Port, HW_Height2_Pin) == GPIO_PIN_SET)
            Height_SWITCH(2) = off;
        else
            //侧边红外
            Height_SWITCH(2) = on;

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
    vTaskDelete(NULL);                                 //从任务列表中移除该任务
    Read_Swicth_tasHandle = NULL;                      //句柄置空
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
 * @Name    Get_Switch_Status
 * @declaration : 获取指定ID开关的通断状态
 * @param   id: [输入/出] 电机编号（1-8）
 * @retval   : 该开关的通断状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_Switch_Status(int id)
{
    if (read_task_exit)
        return err; //确保当前任务处在进行中
    if (id < 1 || id > 10)
        return off; // todo当有开关数量更新时记得修改此处的值
    //上面两个是为了避免出错的判断条件
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
    // todo当有开关数量更新时记得修改此处的值
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
    if (id < 0 || id > 3) // todo当有开关数量更新时记得修改此处的值
        return off;
    return Side_SWITCH(id);
}
int Get_Height_Switch(int id)
{
    if (id < 1 || id > 2)
        return off; // todo 记得在更新元器件数量后更新次此处的限制范围
    return Height_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Height
 * @declaration : 获取此时高度以计算应该调用的动作组编号
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
                 1  为车头朝前   向左扫描   用于条形平台
                 2  为车头朝前   向右扫描   用于条形平台
                 5  为侧身贴边   向前移动   用于阶梯平台
                 6  为侧身贴边   向后移动   用于阶梯平台
**			 enable_imu: [输入/出] 是否使能IMU来保持车身角度的为直线
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void MV_HW_Scan(int color, int dir, int enable_imu)
{
    Set_NeedUp(false);    //禁止高度变换
    MV_Start();           //开启Openmv
    Disable_StopSignal(); //此时不会停车

    Set_IMUStatus(enable_imu); //设置陀螺仪状态
    if (dir == 5 || dir == 6)  //阶梯平台的球
    {
        osDelay(100);
        MV_SendCmd(1, color);    //向openmv发送颜色信号  todo记得改回1
        Action_Gruop(11, 1);     //展开爪子
        Set_IFUP(true);          //设置机械臂动作标志位
        Judge_Side(dir);         //裁决高度判断模式，根据参数给变量赋值 只在阶梯平台需要用到
        Wait_Servo_Signal(5000); //等待舵控信号
        Start_HeightUpdate();    //开始高度判断模式
    }
    else if (dir == 1 || dir == 2) //条形平台
    {
        Openmv_Scan_Bar(1, color); //让OPENMV开始执行条形平台任务
        Wait_Servo_Signal(5000);
    }
    /*1 2为正前方， 5 6为左右侧*/
    if (dir == 1) //使用左边的红外来完成任务
    {
        while (Get_HW_Status(dir) == false)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5); //先移动到此时红外开始扫描到，防止下方直接被跳过
        }
        set_speed(0, 0, 0);
    dir1_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on) //当未收到MV停止信号或红外开持续导通时
        {
            if (Get_HW_Status(Return_ReverseID(dir)) == on) //查看对侧红外开关的状态
                set_speed(MIN_ * 0.8, VERTICAL, 0);         //一边走一边贴边
            else
                set_speed(MIN_ * 0.8, 0, 0); //此时已经有一边出去，防止开关卡死，取消垂直方向的速度，保持水平的速度即可
            osDelay(5);
        }
        set_speed(0, 0, 0);            //停车
        if (Get_HW_Status(dir) == off) //判断退出上述循环的原因，如果是红外触发的，说明此时到达边界，扫描结束，退出函数
        {
            Openmv_Scan_Bar(0, 1);
            Exit_Height_Upadte();
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            //是因为收到MV抓球的信号而停止，等到动作组执行完毕后继续扫描，只单次响应
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //清除停车标志位，此时可以开车
            osDelay(100);         //没啥用，求个心安
            goto dir1_Start_Symbol;
        }
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir) == false)
        {
            set_speed(-MIN_, 0, 0); //横向移动
            osDelay(5);             //先移动到此时红外开始扫描到，防止下方直接被跳过
        }
        set_speed(0, 0, 0);
    dir2_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_HW_Status(dir) == on)
        {
            if (Get_HW_Status(Return_ReverseID(dir)) == on) //查看对侧红外开关的状态
                set_speed(-MIN_ * 0.8, VERTICAL, 0);        //一边走一边贴边
            else
                set_speed(-MIN_ * 0.8, 0, 0); //此时已经有一边出去，防止开关卡死，取消垂直方向的速度，保持水平的速度即可
            osDelay(5);
        }
        set_speed(0, 0, 0);
        if (Get_HW_Status(dir) == off)
        {
            Openmv_Scan_Bar(0, 1);
            Exit_Height_Upadte();
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //清除停车标志位，此时可以开车
            osDelay(100);         //没啥用，求个心安
            goto dir2_Start_Symbol;
        }
    }
    //只有在阶梯平台移动时需要使用高度信息，前面两个用于扫描阶梯平台
    else if (dir == 5)
    {
        while (Get_Side_Switch(1) == off)
        {
            set_speed(0, MIN_, 0);
            osDelay(5); //避免开关卡死
        }
    dir5_Start_Symbol:

        while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
        {
            set_speed(VERTICAL, MIN_ * 0.5, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        if (Get_Side_Switch(1) == off)
        {
            Action_Gruop(4, 1); //收起机械臂
            Set_IFUP(false);
            Exit_Height_Upadte();
            MV_Stop(); //停止处理响应
            Wait_Servo_Signal(Wait_Servo_Done);
            return;
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //清除停车标志位，此时可以开车
            Recover_EnableStatus();
            osDelay(100); //没啥用，求个心安
            goto dir5_Start_Symbol;
        }
    }
    else if (dir == 6)
    {
        while (Get_Side_Switch(2) == off)
        {
            set_speed(0, -MIN_, 0);
            osDelay(5); //避免开关卡死
        }
    dir6_Start_Symbol:
        while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on) //为了多走一点路程，临时改为1号
        {
            set_speed(VERTICAL * 1.5, -MIN_ * 0.5, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);            //停车再说
        if (Get_Side_Switch(2) == off) //是边缘的红外导致的停车
        {
            Action_Gruop(4, 1); //收起机械臂
            Set_IFUP(false);
            Exit_Height_Upadte(); //结束任务
            MV_Stop();            //停止处理响应
            Wait_Servo_Signal(Wait_Servo_Done);
            return; //退出
        }
        else
        {
            Wait_Servo_Signal(Wait_Servo_Done);
            Disable_StopSignal(); //清除停车标志位，此时可以开车
            Recover_EnableStatus();
            osDelay(100);           //没啥用，求个心安
            goto dir6_Start_Symbol; //回到开头位置
        }
    }
}

/**
 * @description: 针对复赛的矩形和二维码的函数
 * @param {int} dir  运行方向 5为从左到右 6为从右到左
 * @param {int} color  目标的颜色 1红2蓝
 * @param {int} QR   是否开启运行
 * @param {int} imu_enable  是否启用陀螺仪辅助角度维持
 * @return {*} null
 */
void Brick_QR_Mode(int dir, int color, int QR, int imu_enable)
{
    Set_NeedUp(QR); //高度变换开启与关闭
    QR_Mode_Init(QR, (QRcolor_t)color);
    MV_Start();                //开启Openmv
    Disable_StopSignal();      //此时不会停车
    osDelay(300);              //等待处理缓冲数据
    Recover_EnableStatus();    //开启mv和二维码工作
    MV_SendCmd(5, color);      //矩形模式，扫方块
    Set_IMUStatus(imu_enable); //控制陀螺仪开关
    if (dir == 5 || dir == 6)
    {
        Action_Gruop(14, 1);     //升起
        Set_IFUP(true);          //标志位
        Judge_Side(dir);         //根据方向设置起始高度对机械臂动作组进行变换
        Wait_Servo_Signal(5000); //等待舵机的完成信号 最多的等待5s
        QR_Mode_Height();        //根据模式进行高度变换 此时的目标高度来自judge_side的赋值
        Start_HeightUpdate();    //因为决定自己执行高度变换 所以不需要在任务开头执行高度变换
        if (dir == 5)
        {
            while (Get_Side_Switch(1) == off)
            {
                set_speed(0, MIN_*0.5, 0); //确保此时开关的工作状态正常
                osDelay(5);            //避免开关卡死
            }
        dir5_Start_Symbol:
            // 换言之 如果高度更新未完成 或者 停车命令被启用 或者 此时已经走出区域（红外开关off）
            while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on && Get_Update_Result())
            {
                set_speed(VERTICAL, MIN_, 0);
                osDelay(5);
            }
            set_speed(0, 0, 0);
            if (Get_Side_Switch(1) == off)
            {
                Action_Gruop(4, 1); //收起机械臂
                Set_IFUP(false);
                Exit_Height_Upadte();
                MV_Stop(); //停止处理响应
                Wait_Servo_Signal(Wait_Servo_Done);
                return;
            }
            else if (Get_Update_Result() == false)
            {
                Wait_Update_finish();
                osDelay(100);
                goto dir5_Start_Symbol; //回到开头位置
            }
            else
            {
                Wait_Servo_Signal(Wait_Servo_Done);
                QR_Mode_Height();
                Disable_StopSignal(); //清除停车标志位，此时可以开车
                Recover_EnableStatus();
                osDelay(100); //没啥用，求个心安
                goto dir5_Start_Symbol;
            }
        }
        else if (dir == 6)
        {
            while (Get_Side_Switch(2) == off)
            {
                set_speed(0, -MIN_*0.5, 0);
                osDelay(5); //避免开关卡死
            }
        dir6_Start_Symbol:
            while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on && Get_Update_Result()) //为了多走一点路程，临时改为1号
            {
                set_speed(VERTICAL, -MIN_ * 0.7, 0);
                osDelay(5);
            }
            set_speed(0, 0, 0);            //停车再说
            if (Get_Side_Switch(2) == off) //是边缘的红外导致的停车
            {
                Action_Gruop(4, 1); //收起机械臂
                Set_IFUP(false);
                Exit_Height_Upadte(); //结束任务
                MV_Stop();            //停止处理响应
                Wait_Servo_Signal(Wait_Servo_Done);
                return; //退出
            }
            else if (Get_Update_Result() == false)
            {
                Wait_Update_finish();
                osDelay(100);
                goto dir6_Start_Symbol; //回到开头位置
            }
            else
            {
                Wait_Servo_Signal(Wait_Servo_Done);
                QR_Mode_Height();
                Disable_StopSignal(); //清除停车标志位，此时可以开车
                Recover_EnableStatus();
                osDelay(100);           //没啥用，求个心安
                goto dir6_Start_Symbol; //回到开头位置
            }
        }
    }
    MV_Stop(); //关闭MV的所有功能 避免此时扫到仓库中物料触发命令
}
/**
 * @name: Kiss_Ass
 * @brief: 倒球使用 屁股对准门
 * @param {int} dir 方向
 * @param {int} enable_imu 陀螺仪使能开关
 * @return {*}
 */
void Kiss_Ass(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu); //测试一下不适用陀螺仪的运行状态
    if (dir == 1)
    {
        while (Get_HW_Status(dir + 2) == off)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir + 2) == on)
        {
            set_speed(-MIN_ * 2, 0, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        // TODO 在此时运行倒下去的动作组，把东西倒下去F
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir + 2) == off)
        {
            set_speed(-MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir + 2) == on)
        {
            set_speed(MIN_ * 2, 0, 0);
            osDelay(5);
        }
        set_speed(0, 0, 0);
        // TODO 在此时运行倒下去的动作组，把东西倒下去
    }
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
    int Switch_Factor = 80;
    int MIN_SPEED = 100;

    if (read_task_exit)
        Start_Read_Switch();

    track_status(1, 0); //关闭循迹版，避免造成方向上的影响
    track_status(2, 0);

    volatile static short flag1, flag2;
    short x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;
    {
        //关于参数的解析，根据方向来判定速度的分配方向
        if (dir == 1) //正Y方向
        {
            w1 = 1, w2 = 2;
            x_pn = 0, y_pn = 1;
        }
        /***下面的已经被废弃
        // else if (dir == 2) //负X方向
        // {
        //     w1 = 4, w2 = 3;
        //     x_pn = -1, y_pn = 0;
        *****/
        else if (dir == 3) //正X方向
        {
            w1 = 3, w2 = 4;
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
    Set_IMUStatus(false);
    flag1 = flag2 = 0;
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
        if (flag1 == on || flag2 == on) //当有开关触碰到时，关闭陀螺仪
            Set_IMUStatus(false);       //关闭陀螺仪,否则设置w速度无意义
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
    Set_InitYaw(0);
    Set_IMUStatus(true);
    set_speed(0, 0, 0); //开关
    /*******本来这里应该接一个矫正陀螺仪，但是会降低程序的灵活性，所以不添加。在调用本程序之后，自己操作陀螺仪*******/
    // todo：调用完函数根据实际需要进行陀螺仪角度的修正
}

/**********************************************************************
 * @Name    HWSwitch_Move
 * @brief  单独使用红外来移动到平台的一侧
 * @param   dir: [输入/出]  贴边移动的方向 1 2 为左右 5 6为侧边的 具体参考阶梯平台
 **			 enable_imu: [输入/出]  是否使能陀螺仪
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void HWSwitch_Move(int dir, int enable_imu)
{
    Set_IMUStatus(enable_imu);
    if (dir == 1)
    {
        while (Get_HW_Status(dir) == off)
        {
            set_speed(-MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir) == on)
        {
            set_speed(-MIN_, VERTICAL, 0);
            osDelay(10);
        }
    }
    else if (dir == 2)
    {
        while (Get_HW_Status(dir) == off)
        {
            set_speed(MIN_, 0, 0);
            osDelay(5);
        }
        while (Get_HW_Status(dir) == on)
        {
            set_speed(MIN_, VERTICAL, 0);
            osDelay(10);
        }
    }
    else if (dir == 5)
    {
        while (Get_Side_Switch(1) == off)
        {
            set_speed(0, MIN_, 0);
            osDelay(5);
        }
        while (Get_Side_Switch(1) == on)
        {
            set_speed(VERTICAL, MIN_, 0);
            osDelay(10);
        }
    }
    else if (dir == 6)
    {
        while (Get_Side_Switch(2) == off)
        {
            set_speed(0, -MIN_, 0);
            osDelay(5);
        }
        while (Get_Side_Switch(2) == on)
        {
            set_speed(VERTICAL, -MIN_, 0);
            osDelay(10);
        }
    }
    set_speed(0, 0, 0);
    osDelay(100);
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
 * @Name    QR_Scan
 * @declaration :使用二维码进行阶梯平台的扫描
 * @param   status: [输入/出]  是否开启
 **			 color: [输入/出]  要抓的颜色，用于判断高度  代表红色 2代表蓝色
 **			 dir: [输入/出]    方向
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void QR_Scan(int status, int color, int dir, int enable_imu)
{
#define OPEN_Claws 1      //爪子对应的舵机，让爪子张开一点避免影响二维码扫描
    int time_delay = 0;   //避免超时卡死的临时变量
    Disable_StopSignal(); //可以开车了
    if (status)           //使能状态
    {
        Set_QR_Status(true);                                    //在这里设置mv和二维码的工作状态
        Set_IMUStatus(enable_imu);                              //经测试，有陀螺仪时容易导致卡死
        Set_QR_Target((QRcolor_t)color);                        //设置目标
        Judge_Side(dir);                                        //根据方向判断起始高度
        Action_Gruop(11, 1);                                    //升起爪子
        while (Get_Servo_Flag() == false && time_delay <= 5000) //等待完成，同时避免超时卡死设置5秒的时间阈值
        {
            time_delay += 10;
            osDelay(10);
        }
        time_delay = 0;       //重新初始化参数
        Set_QR_Status(1);     //开始对二维码的信息读取有反应
        Start_HeightUpdate(); //开始更新高度信息的任务

        if (dir == 5)
        {
            while (Get_Side_Switch(1) == off) //避免卡死
            {
                set_speed(0, MIN_, 0); //先给一个速度一直走 避免初始时开关直接没亮导致任务直接结束
                osDelay(5);
            }
        QR_Scan5_Symbol:
            set_speed(VERTICAL, MIN_, 0); //给一个平行和垂直的速度，一边紧贴着一边移动
            while (Get_Stop_Signal() == false && Get_Side_Switch(1) == on)
                osDelay(5);
            set_speed(0, 0, 0);
            if (Get_Side_Switch(1) == off)
            {
                Action_Gruop(4, 1); //收起机械臂
                Set_QR_Status(0);
                Exit_Height_Upadte();
                return;
            }
            else
            {
                while (Get_Servo_Flag() == false)
                    osDelay(5);
                Disable_StopSignal(); //清除停车标志位，此时可以开车
                osDelay(100);         //没啥用，求个心安
                goto QR_Scan5_Symbol;
            }
        }
        else if (dir == 6)
        {
            while (Get_Side_Switch(2) == off)
            {
                set_speed(0, -MIN_, 0);
                osDelay(5); //避免开关卡死
            }
        QR_Scan6_Symbol:
            set_speed(VERTICAL, -MIN_, 0);
            while (Get_Stop_Signal() == false && Get_Side_Switch(2) == on)
                osDelay(5);
            set_speed(0, 0, 0);
            if (Get_Side_Switch(2) == off)
            {
                Action_Gruop(4, 1); //收起机械臂
                Set_QR_Status(0);   //避免误触发
                Exit_Height_Upadte();
                return;
            }
            else
            {
                while (Get_Servo_Flag() == false)
                    osDelay(5);
                Disable_StopSignal(); //清除停车标志位，此时可以开车
                osDelay(100);         //没啥用，求个心安
                goto QR_Scan6_Symbol;
            }
        }
    }
    else
    {
        Set_QR_Status(0);                                       //关闭二维码的信息处理
        Action_Gruop(4, 1);                                     //收起机械臂
        while (Get_Servo_Flag() == false && time_delay <= 5000) //等待完成，同时避免超时卡死设置5秒的时间阈值
        {
            time_delay += 10;
            osDelay(10);
        }
    }
}
/**
 * @name:
 * @brief:
 * @param {*}
 * @return {*}
 */
void Ring_Move(void)
{
#define Ring_Action 26                      //对应动作组的编号
#define Ring_HW_ID                          //复用的红外的编号
#define Side_Factor 0.3                     //为保持方向给一个垂直速度
    set_speed(MIN_ * Side_Factor, MIN_, 0); //开始动起来
    while (Get_Side_Switch(1) == off)       //复用高度的红外进行位置的判断
        osDelay(1);                         //给系统调度时间
    set_speed(0, 0, 0);                     //到达位置马上停车
    osDelay(1000);                          //给时间用于缓冲
    // Action_Gruop(Ring_Action, 1);           //执行对应的动作组
}
void Disc_Mea(void)
{
    MV_Stop();
    Action_Gruop(25, 1);     //执行预备动作组
    Wait_Servo_Signal(2000); //等待动作组
    printf("\n开始靠近圆盘机\n");
    Wait_Switches(1);    //撞击挡板修正角度以及定位
    Action_Gruop(35, 1); //执行第二个动作组
    printf("\n开始扫描\n");
    Wait_Servo_Signal(2000);      //等待动作组完成
    MV_Start();                   //开启openmv
    MV_SendCmd(6, 1);             // 1红2蓝 3黄
    while (Get_DiscStatus() == 0) //等到9个目标球都抓完
        ;
    printf("\n开始更换目标球颜色\n");
    Action_Gruop(33, 1);     //执行切换仓库动作组
    Wait_Servo_Signal(2000); //等待动作组完成
    MV_SendCmd(6, 3);        //通知MV切换目标颜色
    while (Get_DiscStatus() == 1)
        ;
    printf("\n收起\n");
    Action_Gruop(24, 1);     //结束的动作组
    Wait_Servo_Signal(2000); //等待动作组完成
    MV_Stop();
}
