#include "general.h  "
#include "cmsis_os.h"



/**********************************************************************
  * @Name    Openmv_Scan_Bar
  * @declaration :开启OPENMV扫描条形平台
  * @param   status: [输入/出]  开始还是结束
**			 color: [输入/出] 要抓的颜色，反馈给mv
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Openmv_Scan_Bar(int status, int color)
{
    if(status == 1)
    {
        MV_Start();
        osDelay(100);
        MV_SendCmd(3, color);
        Action_Gruop(5, 1);
    }
    else if(status == 0)
    {
        MV_Stop();
        Action_Gruop(6, 1);

    }
}



