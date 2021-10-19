#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "motor.h"
#include "imu_pid.h"
uint32_t time;

#define LINE_FACTOR 160

#define MAX_SPEED 500
#define MIN_SPEED 120
#define LINE_ERROR_ENCODER 150

static int dir, lines, en_dir, en_val;
static int count_line_status = 1, encodermove_status = 1;
int edge_status[3] = {0};
double bias = 0, variation; //鍙橀噺澹版槑

osThreadId Line_Handle = NULL;       //澹版槑鏁扮嚎鐨勪换鍔″彞锟�???
void LineTask(void const *argument); //澹版槑瀵瑰簲鐨勫彉锟�???

osThreadId Encoder_Handle = NULL; //澹版槑鏁扮嚎鐨勪换鍔″彞锟�???
void EncoderTask(void const *argument);

/**********************************************************************
 * @Name    move_slantly
 * @declaration : 鍊炬枩璧锋
 * @param   dir: [杈撳�?/鍑篯  鏂瑰�? 绗涘崱灏斿潗鏍囩�?4澶ц薄锟�???
 **			 speed: [杈撳�?/鍑篯 鐩爣閫熷害
 **			 delay: [杈撳�?/鍑篯 寤惰繜鐨勬椂闂达紝鐢ㄤ簬椋橀€稿埌绾夸笂
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void move_slantly(int dir, int speed, uint16_t delay)
{
    set_imu_status(true); //寮€鍚檧铻轰华淇濊瘉瑙掑害绋冲�?
    int x_factor, y_factor;
    switch (dir)
    {
    case 1:
        x_factor = 1, y_factor = 1;
        break;
    case 2:
        x_factor = -1, y_factor = 1;
        break;
    case 3:
        x_factor = -1, y_factor = -1;
        break;
    case 4:
        x_factor = 1, y_factor = -1;
        break;
    default:
        x_factor = 0, y_factor = 0;
    }
    set_speed(x_factor * speed, y_factor * speed, 0);
    osDelay(delay);
    set_speed(0, 0, 0);
    osDelay(500);
}
/**********************************************************************
 * @Name    direct_move
 * @brief   閫氳繃寰抗鏁扮嚎鐩寸嚎琛岃�?
 * @param   direct: [杈撳�?/鍑篯 琛岃繘鏂瑰悜
 **		line_num: [杈撳�?/鍑篯  瑕佽蛋鐨勭嚎锟�???
 * @retval
 * @author  peach99CPP
 * @Data    2021-08-06
 ***********************************************************************/

void direct_move(int direct, int line_num, int edge_if, int imu_if)
{
    static int delay_time;
    if (count_line_status) //纭繚涓婁竴涓换鍔″畬鎴愮殑鎯呭喌涓嬶紝鍐嶆墽琛屼笅涓€涓换锟�???
    {
    START_LINE:
        set_imu_status(imu_if);
        //浣跨敤浠诲姟鍒涘缓鐨勫舰寮忔墽琛岃鍑芥�?
        if (direct == 1)
        {
            if (edge_if)
                edge_status[1] = edge_status[2] = 1;
            else
                edge_status[1] = edge_status[2] = 0;
        }
        else if (direct == 2)
        {
            if (edge_if)
                edge_status[0] = 1;
            else
                edge_status[0] = 0;
        }
        count_line_status = 0; //澶栭儴鑾风煡浠诲姟瀹屾垚涓庡惁鐨勪緷锟�????

        dir = direct;
        lines = line_num;
        osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 256);
        Line_Handle = osThreadCreate(osThread(line_task), NULL);
    }
    else
    {
        delay_time = 0;
        while (!count_line_status)
        {
            delay_time++;         //璁℃椂鍙橀�?
            osDelay(100);         //鏈€澶氱�?2绉�
            if (delay_time >= 20) //瓒呮椂锛屼笉鎵ц浠诲姟锛岀洿鎺ラ€€鍑�?
                return;
        }
        goto START_LINE;
    }
}
void LineTask(void const *argument)
{
    int if_need_zero = 0; //鍒ゆ柇鏄惁闇€瑕佽浆鍥炴潵鐨勫彉閲�?
    //寮€鍚笁涓惊杩圭増鐨勫惊杩逛娇鑳藉紑鍏�?
    y_bar.if_switch = true;
    x_leftbar.if_switch = true;
    x_rightbar.if_switch = true;
    while (1) //姝诲惊鐜�?
    {
        static int speed_set = 0;
        static short error;
        if (dir == 1 && lines > 0) //姘村钩鍚戝彸
        {
            y_bar.if_switch = false;    //鍏抽棴Y鏂瑰悜鐨勫惊杩�
            x_leftbar.if_switch = true; //寮€姘村钩鏂瑰悜鐨勫惊杩�?
            x_rightbar.if_switch = true;
            Clear_Line(&x_rightbar); //姘村钩鍚戝彸锛屼娇鐢ㄥ彸渚у惊杩规澘瀛愭潵璁＄畻绾跨殑鏁伴噺
            do
            {
                error = lines - x_rightbar.line_num; //璁＄畻褰撳墠杩樺樊鍑犳牴绾垮埌杈剧洰鏍囩�?
                if (error == 0)
                {
                    set_speed(MIN_SPEED, 0, 0); //褰撳埌杈炬椂锛屼负纭繚杞﹁韩澶勪簬璺彛杈冧负涓棿鐨勪綅缃紝鐢ㄤ綆閫熺户缁璧�
                    while (y_bar.num == 0)
                    {
                        osDelay(5); // y鏂瑰悜瀵昏抗鏉挎湁鐏嵆鍙€€鍑猴紝鍚庨潰寮€鍚惊杩圭増鍗冲彲閫氳繃寰抗浣垮緱杞﹁韩鐭�?
                    }
                    Comfirm_Online(2); //鍚戝彸鐩村埌涓婄�?
                    goto EXIT_TASK;    //浠诲姟缁撴潫
                }
                speed_set = Limit_Speed(LINE_FACTOR * error); //鏅€氭儏鍐典笅锛岀洿鎺ュ璇樊杩涜鏀惧ぇ
                set_speed(speed_set * error, 0, 0);
                osDelay(5);
            } while (error >= 0);
        }
        else if (dir == 1 && lines < 0) //姘村钩鍚戝乏
        {
            //鍚屼笂锛屽紑鍚按骞崇殑寰抗鍗冲彲
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
            lines *= -1;            //鎶婄嚎鏁拌浆鍖栦负姝ｅ€�?
            Clear_Line(&x_leftbar); //閲嶆柊鍒濆鍖栨暟鎹�?
            do
            {
                error = lines - x_leftbar.line_num;
                if (error == 0)
                {
                    set_speed(-MIN_SPEED, 0, 0);
                    while (y_bar.num == 0)
                    {
                        osDelay(5);
                    }
                    Comfirm_Online(1);
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(-speed_set, 0, 0);
                osDelay(5);
            } while (error >= 0);
        }
        else if (dir == 2)
        {
            //鍙紑鍚痀鏂瑰悜鐨勫惊杩�
            y_bar.if_switch = true;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
            if (lines < 0)
            {
                // todo 璁板緱妫€鏌ユ墽琛屽�? 锛屾湁娌℃湁鏈€缁堝洖鍒板垵濮嬭搴�?
                if_need_zero = 1;   //绛変細闇€瑕佸啀杞洖鏉�?
                Turn_angle(1, 180,0); //鍏堣浆寮埌180搴︼紝鐒跺悗鍐嶈繘琛屽墠杩涳紝鍥犱负鍙湁姝ｅ墠鏂规湁寰抗鐗�
                lines *= -1;
            }
            Clear_Line(&y_bar); //閲嶆柊鍒濆鍖栬鐢ㄥ埌鐨勭粨鏋勪綋
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    set_speed(0, MIN_SPEED, 0);
                    while (x_leftbar.num == 0 && x_rightbar.num == 0)
                        osDelay(5);    //鍦ㄦ病鏈夎揪鍒拌矾涓棿鏃讹紝缁х画缁欎竴涓皬閫熷害锛岀洿鍒版按骞冲惊杩圭増涓婃湁鐏€�?
                    Comfirm_Online(3); //缁х画绉诲姩鐩村埌涓婄�?
                    goto EXIT_TASK;
                }
                speed_set = Limit_Speed(LINE_FACTOR * error);
                set_speed(0, speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
    }
EXIT_TASK:
    set_speed(0, 0, 0);    //鍋滆�?
    count_line_status = 1; //鏍囪鏁扮嚎浠诲姟瀹屾�?
    //寮€鍚袱涓柟鍚戠殑寰抗

    //鍏抽棴寰抗鐗堬紝鍚庨潰鍐嶆寜闇€寮€鍚�?
    if (if_need_zero)       //鍒ゆ柇鏄惁闇€瑕佽浆鍥炲師鏉ョ殑瑙掑害
        Turn_angle(1, 180,1); //杞洖鍘熸潵鐨勮搴�?
    else 
    {
        y_bar.if_switch = true;
        x_leftbar.if_switch = true;
        x_rightbar.if_switch = true;
        osDelay(2000); //寮€鍚矾鍙ｇ煫姝� 浣垮緱杞﹁韩姝ゆ椂浣嶄簬璺彛涓ぎ
        y_bar.if_switch = false;
        x_leftbar.if_switch = false;
        x_rightbar.if_switch = false;
    }
    vTaskDelete(NULL);  //閫€鍑轰换鍔�?
    Line_Handle = NULL; //鍙ユ焺缃┖
}

/**********************************************************************
 * @Name    move_by_encoder
 * @鍔熻兘璇存槑  璁＄畻缂栫爜鍣ㄥ€艰绠楄窛绂伙紝闈㈠渚у悜绉诲姩鏃堕渶瑕佹坊鍔犺浆锟�???
 * @param   val: [杈撳�?/鍑篯  杈撳叆绉诲姩鐨勶�??
 * @杩斿洖锟�????
 * @author  peach99CPP
 ***********************************************************************/
/*9.14淇敼璁板綍锛屽皢鍑芥暟杩愯鏂瑰紡鏀逛负杩涚▼妯″紡锛屽皾璇曡В鍐崇浉鍚岀Щ鍔ㄦ暟鍊肩殑闂�?*/
void move_by_encoder(int direct, int val)
{
    static int encoder_delay;
    if (encodermove_status) //涓婁竴涓换鍔¤繍琛岀粨鏉燂紝鎵嶅彲浠ュ紑濮嬭繍琛屼笅涓€涓换鍔★紝閬垮厤鍑洪敊
    {
    START_ENCODER:
        set_imu_status(true); //纭繚闄€铻轰华寮€鍚紝璇曢獙鎬э紝涓嶇‘瀹氳涓嶈

        en_dir = direct; //灏嗗弬鏁颁紶閫掔粰鍏ㄥ眬鍙橀噺锟�????
        en_val = val;

        encoder_sum = 0; //灏嗙紪鐮佸櫒绱姞鍊肩疆0
        //寮€鍚换锟�????
        osThreadDef(encodermove, EncoderTask, osPriorityRealtime, 0, 256); //浠诲姟浼樺厛绾х粰鍒版渶楂橈紝纭繚鍙婃椂鍝嶅�?
        Encoder_Handle = osThreadCreate(osThread(encodermove), NULL);
        //浠诲姟缁撴潫鏍囧�?
        encodermove_status = 0;
    }
    else
    {
        encoder_delay = 0;
        while (!encodermove_status)
        {
            encoder_delay++;
            osDelay(100);
            if (encoder_delay >= 20)
                return;
        }
        goto START_ENCODER;
    }
}
void EncoderTask(void const *argument)
{
#define ENOCDER_DIVIDE_FACTOR 50
#define ENCODE_THRESHOLD 2
#define ENCODER_FACTOR 8
    clear_motor_data();
    time = TIME_ISR_CNT; //鑾峰彇绯荤粺鏃堕�?
    if (en_dir == 1)
    {
        y_bar.if_switch = false; //鍏抽棴涓€渚х殑瀵昏抗锟�????
        x_leftbar.if_switch = true;
        x_rightbar.if_switch = true;

        if (en_val < 0) //鍚戝�?
        {
            en_val *= -1;
            while (1) //鏈埌杈剧洰锟�???
            {
                if ((TIME_ISR_CNT - time > 50) && ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                    goto Encoder_Exit;                                       //瓒呮椂澶勭悊锛岄伩鍏嶅崱锟�???
                bias = -ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)); //寰楀埌宸拷?
                variation = bias * ENCODER_FACTOR;                           //璁＄畻寰楀嚭杈撳嚭鍊笺€侾锟�???
                variation = Limit_Speed(variation);                          //鍒嗛厤鏈€浣庨€熷害锛岄伩鍏嶅崱锟�???
                set_speed(variation, 0, 0);                                  //鍒嗛厤閫熷害

                osDelay(5); //缁欎换鍔¤皟搴﹀唴鏍稿垏鎹㈢殑鏈轰細
            }
        }
        else
        {
            //鍚戝彸涓烘
            while (1)
            {
                if ((TIME_ISR_CNT - time > 50) && ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                    goto Encoder_Exit;
                bias = ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
                variation = bias * ENCODER_FACTOR;
                variation = Limit_Speed(variation);
                set_speed(variation, 0, 0);
                osDelay(5);
            }
        }
    }
    else if (en_dir == 2)
    {
        static int pn;
        y_bar.if_switch = true; //鍏抽棴鍗曚晶鐨勫杩规澘
        x_leftbar.if_switch = false;
        x_rightbar.if_switch = false;
        if (en_val < 0)
        {
            pn = -1;      //鏍囪鍏朵负璐熸�?
            en_val *= -1; //缁濆鍊煎寲
        }
        else
            pn = 1;
        while (1) //鍚屼�?
        {
            if ((TIME_ISR_CNT - time > 50) && ABS(en_val - (encoder_sum / ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD)
                goto Encoder_Exit;
            bias = fabs(ABS(en_val) - (encoder_sum / ENOCDER_DIVIDE_FACTOR));
            variation = bias * ENCODER_FACTOR;
            variation = Limit_Speed(variation);
            set_speed(0, variation * pn, 0);
            osDelay(5);
        }
    }
Encoder_Exit:
    set_speed(0, 0, 0);     //鍋滆�?
    encodermove_status = 1; //鏍囪缁撴潫锟�???
    vTaskDelete(NULL);      //浠庝换鍔″垪琛ㄤ腑灏嗗叾绉诲�?
    Encoder_Handle = NULL;  //灏嗘寚閽堟寚鍚戠�?
}

/**********************************************************************
 * @Name    car_shaking
 * @declaration : 涓€涓瘯楠屾€х殑娴嬭瘯鍔熻兘锛屽彲鑳界敤浜庡嵏璐ф椂纭繚璐х墿琚敥涓嬫�?
 * @param   direct: [杈撳�?/鍑篯 寰€鍝竴涓柟鍚戣繘琛屾憞锟�???
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void car_shaking(int direct)
{
    while (y_bar.num == 0)
    {
        w_speed_set(40);
        osDelay(100);
        if (y_bar.num != 0)
            break;
        w_speed_set(-40);
        osDelay(100);
    }
}

/**********************************************************************
 * @Name    get_count_line_status
 * @declaration : 澶栭儴鑾风煡鏁扮嚎杩愯鐨勫畬鎴愭儏鍐电殑鎺ュ彛
 * @param   None
 * @retval   : 鏄惁杩愯瀹屾�?
 * @author  peach99CPP
 ***********************************************************************/
int get_count_line_status(void)
{
    return count_line_status;
}
/**********************************************************************
 * @Name    get_enocdermove_status
 * @declaration : 澶栭儴鑾风煡闈犵紪鐮佸櫒杩愯鐨勫畬鎴愭儏鍐电殑鎺ュ�?
 * @param   None
 * @retval   :  鏄惁杩愯缁撴�?
 * @author  peach99CPP
 ***********************************************************************/
int get_enocdermove_status(void)
{
    return encodermove_status;
}

/**********************************************************************
 * @Name    Limit_Speed
 * @declaration :瀵硅緭鍏ョ殑瑙掑害鍊艰繘琛屽弻鍚戦檺骞呭悗杈撳嚭
 * @param   speed: [杈撳�?/鍑篯 寰呴檺骞呯殑閫熷害锟�???
 * @retval   :  闄愬箙杩囧悗鐨勯€熷害鍊硷紝鏃笉澶珮鍙堜笉澶綆
 * @author  peach99CPP
 ***********************************************************************/
int Limit_Speed(int speed)
{
    if (speed > 0)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
        if (speed < MIN_SPEED)
            speed = MIN_SPEED;
    }
    else
    {
        if (speed < -MAX_SPEED)
            speed = -MAX_SPEED;

        if (speed > -MIN_SPEED)
            speed = -MIN_SPEED;
    }
    return speed;
}
void Comfirm_Online(int dir)
{
#define LOW_SPEED_TO_CONFIRM 80
    if (dir == 1)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(-LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            track_status(1, 1);
            osDelay(500);
        }
    }
    else if (dir == 2)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            track_status(1, 1);
            osDelay(500);
        }
    }
    else if (dir == 3)
    {
        if (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
        {
            set_speed(0, LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            track_status(1, 1);
            osDelay(500);
        }
    }
}
