#ifndef __PID_H
#define __PID_H

#define NULL 0
typedef unsigned char uint8_t;
typedef  int  Testime;
typedef struct pid_controler
{
    //期望
    float last_expect;
    //期望
    float expect;
    //反馈值
    float feedback;
    //偏差
    float err;
    //上次偏差
    float last_err;
    //上上次偏差
    float pre_last_err;
    //偏差限幅值
    float err_max;
    //积分分离偏差值
    float integrate_separation_err;
    //积分值
    float integrate;
    //积分限幅值
    float integrate_max;
    //偏差微分
    float dis_err;
    //控制参数kp
    float kp;
    //控制参数ki
    float ki;
    //控制参数kd
    float kd;
    //前馈控制参数kdkp
    float feedforward_kp;
    //前馈控制参数kd
    float feedforward_kd;
    //控制器总输出
    float control_output;
    //输出限幅
    float control_output_limit;
    //短路标志
    uint8_t short_circuit_flag;
    //间隔时间计算
    Testime pid_controller_dt;
    //私有数据
    void *pri_data;

    //自定义计算偏差，偏差积分回调
    void (*err_callback)(struct pid_controler *);
} pid_controler_t;

extern float pid_control(pid_controler_t *controler);

#endif

