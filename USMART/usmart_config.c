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
#include "servo.h"
#include "general.h"
#include "avoid_obs.h"
/****各个调试模式的开关*****/

#define DEBUG_MOTOR 0   //调试电机
#define DEBUG_TRACKER 0 //调试循迹版
#define DEBUG_IMU 0     //调试陀螺仪
#define DEBUG_CHASSIS 1 //底盘运动
#define DEBUG_SWITCH 0  //轻触开关、红外开关
#define DEBUG_OPENMV 0  // openmv通讯
#define Debug_Servo 1   //舵控通讯
extern void Global_Debug(void);
/******将显示界面变得更加整洁****/
//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串
struct _m_usmart_nametab usmart_nametab[] =
    {
#if USMART_USE_WRFUNS == 1 //如果使能了读写操作
        (void *)read_addr,
        "u32 read_addr(u32 addr)",
        (void *)write_addr,
        "void write_addr(u32 addr,u32 val)",
#endif
#if DEBUG_MOTOR == 1
        (void *)set_debug_motor,
        "void set_debug_motor(int status, int motor_id)",
        (void *)set_debug_speed,
        "void set_debug_speed(int speed)",
        (void *)set_motor_pid,
        "void set_motor_pid(int kp, int ki, int kd)",
        (void *)set_motor_maxparam,
        "void set_motor_maxparam(int integrate_max, int control_output_limit)",
#endif
#if DEBUG_TRACKER == 1
        (void *)set_track_pid,
        "void set_track_pid(int kp, int ki, int kd)",
        (void *)track_status,
        "void track_status(int id, int status)",
#endif
#if DEBUG_IMU == 1
        (void *)set_imu_param,
        "void set_imu_param(int p,int i,int d)",

/**陀螺仪部分**/
#endif
#if DEBUG_OPENMV == 1
        (void *)Openmv_Scan_Bar,
        "void Openmv_Scan_Bar(int status,int color)",

#endif
#if DEBUG_CHASSIS == 1
        /**底盘运动部分**/
        (void *)Comfirm_Online,
        "void Comfirm_Online(int dir)",
        (void *)move_slantly,
        "void move_slantly(int dir, int speed, uint16_t delay)",
/***目前用不到
(void *)set_motor,
"void set_motor(int motor_id, int control_val)",
***/
#endif
#if DEBUG_SWITCH == 1
        /***轻触开关部分***/
        (void *)QR_Scan,
        "void QR_Scan(int status, int color, int dir, int enable_imu)",
        (void *)MV_HW_Scan,
        "void MV_HW_Scan(int color, int dir, int enable_imu)",
#endif
#if Debug_Servo == 1

        (void *)Ass_Door, //倒球时屁股上的门
        "void Ass_Door(int status)",
        (void *)Lateral_infrared, //侧边红外的开关
        "void Lateral_infrared(int status)",
        (void *)Baffle_Control, //控制挡板
        "void Baffle_Control(int up_dowm)",
        (void *)Single_Control, //控制单个舵机
        "void Single_Control(int id, int control_mode, int angle, int  time, int delay)",
        (void *)Action_Gruop, //控制动作组的执行
        "void Action_Gruop(int id, int  times)",
#endif
        //运动部分
        (void *)set_speed,
        "void set_speed(int x, int y, int w)",
        (void *)move_by_encoder,
        "void move_by_encoder(int  direct, int val)",
        (void *)direct_move,
        "void direct_move(int direct, int line_num, int edge_if,int imu_if)",
        // mv部分
        (void*)MV_Start,
        "void MV_Start(void)",
        (void*)MV_Stop,
        "void MV_Stop(void)",
        (void *)MV_SendCmd,
        "void MV_SendCmd(const uint8_t event_id, const int param)",
        //陀螺仪
        (void *)set_imu_status,
        "void set_imu_status(int status)",
        (void *)Set_InitYaw,
        "void Set_InitYaw(int target)",
        (void *)Turn_angle,
        "void Turn_angle(int mode, int angle, int track_enabled)",
        //开关运用
        (void *)Wait_Switches, //撞击挡板来修正角度
        "void Wait_Switches(int dir)",
        (void *)HWSwitch_Move, //利用边缘的红外进行定位
        "void HWSwitch_Move(int dir,int enable_imu)",
        (void *)Brick_QR_Mode, //阶梯平台
        "void Brick_QR_Mode(int dir, int color, int QR, int imu_enable)",
        (void *)Kiss_Ass, //屁股对准
        "void Kiss_Ass(int dir,int enable_imu)",
        (void *)Ring_Move, //移动到圆环
        "void Ring_Move(void)",
        (void *)Disc_Mea,
        "void Disc_Mea(void)",
        //综合
        (void *)Wait_For_Avoid, //避障
        "void Wait_For_Avoid(int dir)",
        (void *)Global_Debug, //跑路线
        "void Global_Debug(void)",
        //临时添加 todo 后续记得进行移除
        (void*)Trans_Cons,
        "void Trans_Cons(int val1,int val2)",

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
        0,                                                         //参数数量
        0,                                                         //函数ID
        1,                                                         //参数显示类型,0,10进制;1,16进制
        0,                                                         //参数类型.bitx:,0,数字;1,字符串
        0,                                                         //每个参数的长度暂存表,需要MAX_PARM个0初始化
        0,                                                         //函数的参数,需要PARM_LEN个0初始化
};
