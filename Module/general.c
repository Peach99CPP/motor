#include "general.h  "
void Openmv_Scan_Bar(int status,int color)
{
    if(status == 1)
    {
        MV_Start();
        MV_SendCmd(3,color);
        Action_Gruop(5,1);
    }
    else if(status == 0)
    {
        MV_SendCmd(4,color);
        Action_Gruop(6,1);
    }
}


