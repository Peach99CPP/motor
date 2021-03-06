#include "track_bar_receive.h "
#include "usart.h"
#include "atk_imu.h"
#include <string.h>
#include "uart_handle.h"

extern int edge_status[3];
extern volatile uint32_t TIME_ISR_CNT;

#define START_BYTE 0xff
#define END_BYTE 0x99
#define MAX_LINE 3
#define BUFF_SIZE 7
#define LINE_DELAY (50 / 10)

int dma_count, times_counts = 0;

//权重
float track_weight[8] = {4, 3, 2, 1,
                         -1, -2, -3, -4};

uint8_t rec_data, now_id = 0;            //这两个变量在接收中断中使用
trackbar_t y_bar, x_leftbar, x_rightbar; //三个寻迹板结构体变量
//初始化PID参数
pid_paramer_t track_pid_param = // pid参数
    {
        .integrate_max = 50,
        .kp = 12,
        .ki = 0,
        .kd = 0,
        .control_output_limit = 300};

//初始化结构体
Track_RXRows_t Track_Row =
    {
        .track_uart = &huart6,
        .current_index = 0,
        .done_flag = 0,
        .start_flag = 0,
        .rec_data = {0}};

void Update_TRckerCounter(void)
{
    times_counts = times_counts > 32758 ? 0 : times_counts + 1;
    dma_count--;
}
/**********************************************************************
 * @Name    Clear_Line
 * @declaration : 清除结构体的标志位
 * @param   bar: [输入/出] 操作对象
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Clear_Line(trackbar_t *bar)
{
    bar->line_flag = 0;
    bar->line_num = 0;
}

int Get_Current_RowID(void)
{
    return now_id;
}

/**********************************************************************
 * @Name    Get_EmptyRow_ID
 * @declaration : 获取空行的下标
 * @param   now_id: [输入/出]  输入值  优先根据当前值选择其下一个
 * @retval   : 可用下标
 * @author  peach99CPP
 ***********************************************************************/
int Get_EmptyRow_ID(int now_id)
{
    //首先处理一下当前的数据，看看下一个是否可用
    if (now_id + 1 == MAX_ROW_SIZE)
        now_id = 0;
    else
    {
        now_id += 1; //没有到达边界，就直接取下一个
    }

    if (Track_Row.rec_data[now_id][0] == 0) //如果下一个可用，就直接用 否则再重新遍历
        return now_id;
    else
    {
        //遍历二维数组
        for (uint8_t i = 0; i < MAX_ROW_SIZE; ++i)
        {
            if (Track_Row.rec_data[i][0] == 0)
            {
                return i;
            }
        }
    }
    return 0X01; //避免卡死
}

/**********************************************************************
 * @Name    Get_AvaibleRow_ID
 * @declaration :获取有数据的行号下标
 * @param   None
 * @retval   : 装载了有效数据的下标
 * @author  peach99CPP
 ***********************************************************************/
int Get_AvaibleRow_ID(int now_row)
{
    //优先根据当前下标寻找
    if (now_row == MAX_ROW_SIZE) //已经到达尽头
    {
        now_row = 0;                                //那么下一个就是0
        if (Track_Row.rec_data[now_row][0] == 0X01) //检查是否可用
            return now_row;
    }
    else
    {
        //大部分数据
        now_row += 1;                               //直接取下一位
        if (Track_Row.rec_data[now_row][0] == 0X01) //判断是否已经装载了数据
            return now_row;
        else //上述方法未找到 被迫遍历
        {
            for (uint8_t i = 0; i < MAX_ROW_SIZE; ++i) //从头遍历到尾
            {
                if (Track_Row.rec_data[i][0] == 0X01) //检查
                {
                    return i;
                }
            }
        }
    }
    return 0X01; //实在找不到了，随便返回一个，防止被卡死
}

/**********************************************************************
 * @Name    set_track_pid
 * @declaration : 调试寻迹板的PID参数API
 * @param   kp: [输入/出] 放大10倍的p
 **			 ki: [输入/出] 放大10倍的i
 **			 kd: [输入/出] 放大10倍的d
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void set_track_pid(int kp, int ki, int kd)
{
    track_pid_param.kp = kp / 10.0;
    track_pid_param.ki = ki / 10.0;
    track_pid_param.kd = kd / 10.0;
}

/**********************************************************************
 * @Name    track_bar_init
 * @declaration : 对寻迹板需要的相关变量进行初始化的设置
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void track_bar_init(void) //相关的初始化函数
{
    dma_count = 0;
    //变量参数的初始化
    y_bar.id = forward_bar; //标注身份
    y_bar.line_num = 0;
    y_bar.num = 0;
    y_bar.line_num = 0;
    y_bar.data.expect = 0;  //循迹的目标值恒为0
    y_bar.if_switch = true; //使能

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
    __HAL_UART_ENABLE_IT(Track_Row.track_uart, UART_IT_RXNE);
}

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
#define EDGE_THRESHOLD 5 //在边缘数线模式下，几颗灯亮起时为有效计数
#define NUM_THRESHOLD 6  //非边缘线计算下，判断到达线的数量
#define MIN_NUM 2        //在压线之后，过线了才会计算一根线，根据灯的数量进行计数
#define EDGE_VAL 7       //边缘数线状态下的循迹读回来的值

    Update_TRckerCounter();                         //更新相关的计数器
    static uint8_t led_num = 0;                     //计算灯数量的变量
    static uint8_t AVaiable_Row;                    //可用的下标
    AVaiable_Row = Get_AvaibleRow_ID(AVaiable_Row); //进行位置更新
    float track_value = 0, temp_val;                //相关的变量声明
    if (AVaiable_Row != 0XFF)
    {
        //下面的和检验看情况开启
        if ((uint8_t)(Track_Row.rec_data[AVaiable_Row][1] + Track_Row.rec_data[AVaiable_Row][3] + Track_Row.rec_data[AVaiable_Row][5]) == Track_Row.rec_data[AVaiable_Row][6]) //和校验
        {
            for (uint8_t bar_id = 1; bar_id <= BUFF_SIZE - 2; bar_id += 2)
            {
                track_value = 0;
                temp_val = 0;
                led_num = 0;
                for (uint8_t i = 0; i < 8; ++i)
                {
                    temp_val = (bool)(((Track_Row.rec_data[AVaiable_Row][bar_id] << i) & 0x80)) * track_weight[i]; //根据灯亮与否及其权重得到反馈值
                    if (temp_val != 0)
                        led_num++; //计算亮的灯数量
                    track_value += temp_val;
                }
                switch (Track_Row.rec_data[AVaiable_Row][bar_id - 1]) //判断寻迹板ID
                {
                case 1:
                    y_bar.data.feedback = track_value; //赋值
                    y_bar.num = led_num;               //得到灯的数量
                    if (y_bar.num >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
                    {
                        /*有两种情况，
                         *一是当跑在非边缘时，此时灯的数量比较多
                         *二是在边缘跑时，此时灯数量较少且需要加多重判断，比如计算此时的反馈值，一般来说此时的反馈值会比较大。可以以此为判断条件
                         */
                        y_bar.line_flag = 1; //此时到线上
                    }
                    if (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL) //边缘数线的情况下，特殊处理
                        y_bar.data.feedback = 0;                                                               //要放在对线的判断之后；置0是为了防止此时发生偏移
                    if (y_bar.line_flag && y_bar.num <= MIN_NUM)
                    {
                        //使用此机制为了避免因停留在线上而导致线的数量一直重复计数
                        y_bar.line_flag = 0; //数线完成
                        y_bar.line_num++;    //线数目加一
                    }
                    break;
                case 2:
                    x_leftbar.data.feedback = track_value;
                    x_leftbar.num = led_num;

                    if (x_leftbar.num >= NUM_THRESHOLD || (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL))
                    {
                        x_leftbar.line_flag = 1; //标记到了线上
                    }
                    if (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL)
                        x_leftbar.data.feedback = 0;
                    if (x_leftbar.line_flag && x_leftbar.num <= MIN_NUM) //避免因为在线上停留而导致的重复计数问题
                    {
                        x_leftbar.line_flag = 0;
                        x_leftbar.line_num++;
                    }
                    break;
                case 3:
                    x_rightbar.data.feedback = track_value;
                    x_rightbar.num = led_num;
                    if (x_rightbar.num >= NUM_THRESHOLD || (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL))
                    {
                        x_rightbar.line_flag = 1;
                    }
                    if (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL)
                        x_leftbar.data.feedback = 0;
                    if (x_rightbar.line_flag && x_rightbar.num <= MIN_NUM)
                    {
                        x_rightbar.line_flag = 0;
                        x_rightbar.line_num++;
                    }
                    break;
                default: //啥也不干
                    ;
                }
            }
        }
        memset(Track_Row.rec_data[AVaiable_Row], 0, sizeof(Track_Row.rec_data[AVaiable_Row])); //清除此位置的数据内容
    }
}

/**********************************************************************
 * @Name    Track_RX_IRQ
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Track_RX_IRQ(void)
{

    if (__HAL_UART_GET_IT(Track_Row.track_uart, UART_IT_RXNE)) //接收到数据
    {
        rec_data = Track_Row.track_uart->Instance->RDR; //获取数据内容
        if (Track_Row.done_flag == 0)                   //未接收完成
        {

            if (Track_Row.start_flag == 0) //还没有收到首字节
            {
                if (rec_data == START_BYTE) //收到了，泪目！
                {
                    Track_Row.current_index = 0; //重置下标 准备接收
                    Track_Row.start_flag = 1;    //设置标志位
                    return;                      //后面的判断于此无关 直接退出
                }
            }
            else //已经收到了首字节，此时就是正文还有接收完成的处理
            {
                if (rec_data == END_BYTE && Track_Row.current_index >= 7) //接收到末尾字节
                {
                    now_id = Get_EmptyRow_ID(now_id); //更新装载对象的下标
                    Track_Row.done_flag = 1;          //标志接收完成
                    dma_count++;                      //任务运行
                    Track_Row.current_index = 0;      //重置下标
                    Track_Row.start_flag = 0;         //标志为未收到首字节
                    Track_Row.done_flag = 0;          //标志为未结束
                }
                else //正文内容
                {
                    Track_Row.rec_data[now_id][Track_Row.current_index++] = rec_data;              //装载进缓存
                    if (Track_Row.current_index + 2 >= MAX_TRACK_REC_SIZE)                         //防止错误造成的卡死
                    {                                                                              //加2是为了提前
                        Track_Row.current_index = 0;                                               //清除下标 重新开始接收
                        Track_Row.done_flag = 0;                                                   //标志位管理
                        Track_Row.start_flag = 0;                                                  //标志位管理
                        memset(Track_Row.rec_data[now_id], 0, sizeof(Track_Row.rec_data[now_id])); //将内容全部清空
                    }
                }
            }
        }
    }
}

/**********************************************************************
 * @Name    track_pid_cal
 * @declaration : 寻迹板pid计算函数
 * @param   bar: [输入/出] 哪一个寻迹板
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
float track_pid_cal(trackbar_t *bar)
{
    if (bar->if_switch == true) //使能，计算pid值并进行返回
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0; //未使能，不做改变
}

/**********************************************************************
 * @Name    track_status
 * @declaration :设置寻迹板状态
 * @param   id: [输入/出]  方向。1为垂直 ，2为水平
 **			 status: [输入/出]  开启或关闭
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void track_status(int id, int status)
{
    if (id == 1) // y方向
        y_bar.if_switch = status;
    else if (id == 2) // x方向
    {
        x_leftbar.if_switch = status;
        x_rightbar.if_switch = status;
    }
}

/**********************************************************************
 * @Name    Get_Trcker_Num
 * @declaration :
 * @param   bar: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
int Get_Trcker_Num(trackbar_t *bar)
{
    return bar->num;
}
