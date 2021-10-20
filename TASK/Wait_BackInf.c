#include "Wait_BackInf.h "
#include "uart_handle.h"
long wait_time = 0, now_time; //计时器
bool OverTimeFlag = 1;        //会否超时的标志

osThreadId OverTimeHandle = NULL;             //任务句柄
void CountTimeTaskFunc(void const *argument); //任务实现函数
bool CountTimeTask_Exit = 1;                  //是否退出

/**********************************************************************
 * @Name    Get_TimeResult
 * @declaration :获取超时结果
 * @param   None
 * @retval   :是否超时
 * @author  peach99CPP
 ***********************************************************************/
bool Get_TimeResult(void)
{
  return OverTimeFlag;
}

/**********************************************************************
 * @Name    Deinit_Time
 * @declaration :重新初始化计时器相关变量，删除任务前使用
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Deinit_Time(void)
{
  CountTimeTask_Exit = 1;   //标志退出 此时任务结束有可能是因为超时，需要手动置1
  now_time = wait_time = 0; //清零计数器
}

/**********************************************************************
 * @Name    Start_CountTime
 * @declaration : 开始计时
 * @param   timelength: [输入/出] 时间长度
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Start_CountTime(long timelength) //开始任务
{
  if (CountTimeTask_Exit && OverTimeHandle == NULL) //当上一个任务运行结束时
  {
    CountTimeTask_Exit = 0; //设置退出标志
    now_time = 0;           //清零当前计时器
    OverTimeFlag = 0;       //清除超时标志位
    wait_time = timelength;//输入要等待的时间，单位ms
    osThreadDef(CountTimeTask, CountTimeTaskFunc, osPriorityHigh, 0, 256); //定义任务结构体
    OverTimeHandle = osThreadCreate(osThread(CountTimeTask), NULL);        //创建任务
  }
  else 
  printf("计时任务开启失败\r\n");
}

/**********************************************************************
 * @Name    CountTimeTaskFunc
 * @declaration :任务实现函数
 * @param   argument: [输入/出] 无
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void CountTimeTaskFunc(void const *argument)
{
  while (!CountTimeTask_Exit && !OverTimeFlag) //未超时以及未退出时
  {
    while (now_time < wait_time && !CountTimeTask_Exit) //计时过程
    {
      now_time += 5; //累计时间值
      osDelay(5);    //操作系统延迟 进行任务切换
    }
    OverTimeFlag = 1; //标志此时已经超时
    osDelay(10);
  }
  Deinit_Time();         //重新初始化相关变量
  OverTimeHandle = NULL; //运行结束  将句柄置空
  vTaskDelete(NULL);     //删除任务
}

/**********************************************************************
 * @Name    Exit_CountTime
 * @declaration : 退出计时
 * @param   : [输入/出] 无
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Exit_CountTime(void)
{
  if (OverTimeHandle != NULL)
  {
    CountTimeTask_Exit = 1; //标志退出
    wait_time = now_time = 0;
    OverTimeFlag = 1; //避免此时外部卡死在读取
  }
}

int Get_CountTimeExit(void)
{
  return CountTimeTask_Exit;
}
