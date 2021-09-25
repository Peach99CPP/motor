/************************************************************************
  *      
  * FileName   : atk_imu.h   
  * Version    : v1.0		
  * Author     : 桃子			
  * Date       : 2021-09-08         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
*******************************************************************************/



 #ifndef __ATK_IMU_H_
#define __ATK_IMU_H_

#include "cmsis_os.h"
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "math.h"





#define imu901_uart_receive(data, len)	1 /*!< 串口获取接收数据API */
#define imu901_uart_send(data, len)		void	/*!< 串口i发送数据API */

/*模块上行帧头*/
#define UP_BYTE1 		0x55
#define UP_BYTE2 		0x55	/*!< 模块主动上传包头2 */
#define UP_BYTE2_ACK 	0xAF	/*!< 模块指令应答上传包头2 */


/* 模块主动上传指令ID */
enum upmsgid
{
    UP_ATTITUDE		= 0x01,		/*!< 姿态角 */
    UP_QUAT			= 0x02,		/*!< 四元�? */
    UP_GYROACCDATA	= 0x03,		/*!< 陀螺仪加速度 */
    UP_MAGDATA		= 0x04,		/*!< 磁场 */
    UP_BARODATA		= 0x05,		/*!< 气压�? */
    UP_D03DATA		= 0x06,		/*!< 端口状�? */
};

#ifdef REG_Action
/* 寄存器列�? */
enum regTable
{
    REG_SAVE	= 0x00,			/*!< 保存参数命令			（W�? 	*/
    REG_SENCAL	= 0x01,			/*!< 校准命令			（W�? 	*/
    REG_SENSTA	= 0x02,			/*!< 校准状�?			（R�? 	*/
    REG_GYROFSR	= 0x03,			/*!< 陀螺仪量程			（R/W�? */
    REG_ACCFSR	= 0x04,			/*!< 加速度量程			（R/W�? */
    REG_GYROBW	= 0x05,			/*!< 陀螺仪带宽			（R/W�? */
    REG_ACCBW	= 0x06,			/*!< 加速度带宽			（R/W�? */
    REG_BAUD	= 0x07,			/*!< 波特�?			（R/W�? */
    REG_UPSET	= 0x08,			/*!< 回传内容设置			（R/W�? */
    REG_UPSET2	= 0x09,			/*!< 回传内容设置2		（R/W�? */
    REG_UPRATE	= 0x0A,			/*!< 回传速率			（R/W�? */
    REG_ALG		= 0x0B,			/*!< 算法选择			（R/W�? */
    REG_ASM		= 0x0C,			/*!< 安装方式			（R/W�? */
    REG_GACOFF	= 0x0D,			/*!< 陀螺仪自校�?			（R/W�? */
    REG_BACOFF	= 0x0E,			/*!< 气压计自校准			（R/W�? */
    REG_LEDOFF	= 0x0F,			/*!< LED开�?			（R/W�? */
    REG_D0MODE	= 0x10,			/*!< 端口D0模式			（R/W�? */
    REG_D1MODE	= 0x11,			/*!< 端口D1模式			（R/W�? */
    REG_D2MODE	= 0x12,			/*!< 端口D2模式			（R/W�? */
    REG_D3MODE	= 0x13,			/*!< 端口D3模式			（R/W�? */
    REG_D0PULSE	= 0x14,			/*!< 端口D0PWM脉宽		（R/W�? */
    REG_D1PULSE	= 0x16,			/*!< 端口D1PWM脉宽		（R/W�? */
    REG_D2PULSE	= 0x18,			/*!< 端口D2PWM脉宽		（R/W�? */
    REG_D3PULSE	= 0x1A,			/*!< 端口D3PWM脉宽		（R/W�? */
    REG_D0PERIOD	= 0x1C,		/*!< 端口D0PWM周期		（R/W�? */
    REG_D1PERIOD	= 0x1F,		/*!< 端口D1PWM周期		（R/W�? */
    REG_D2PERIOD	= 0x21,		/*!< 端口D2PWM周期		（R/W�? */
    REG_D3PERIOD	= 0x23,		/*!< 端口D2PWM周期		（R/W�? */

    REG_RESET   = 0x7F,			/*!< 恢复默认命令			（W�? 	*/
};
void atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen);
uint8_t atkpReadReg(enum regTable reg, int16_t *data);
#endif

#define ATKP_MAX_DATA_SIZE 28

/*通讯数据结构*/
typedef struct
{
    uint8_t startByte1;
    uint8_t startByte2;
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[ATKP_MAX_DATA_SIZE];
    uint8_t checkSum;
} atkp_t;


/* 姿态角数据结构�? */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

/* 四元数数据结构体 */
typedef struct
{
    float q0, q1, q2, q3;

} quaternion_t;

/* 陀螺仪加速度数据结构�? */
typedef struct
{
    int16_t gyro[3]; 	/*!< 陀螺仪原始数据 */
    int16_t acc[3];		/*!< 加速度原始数据 */
    float 	fgyroD[3];	/*!< 陀螺仪转速�?/S */
    float 	faccG[3];	/*!< 加速度重力 G */
} gyroAcc_t;

#ifdef IMU901
/* 磁力计数据结�? */
typedef struct
{
    int16_t mag[3];		/*!< 磁场数据 */
    float  	temp;		/*!< 磁力计温�? */
} mag_t;

/* 气压计数据结�? */
typedef struct
{
    int32_t pressure;	/*!< 气压值Pa */
    int32_t altitude;	/*!< 海拔高度cm */
    float  	temp;		/*!< 气压计温�? */
} baro_t;
extern mag_t			magData;
extern baro_t			baroData;
#endif

/* 端口数据结构 */
typedef struct
{
    uint16_t d03data[4];
} ioStatus_t;



/* 部分寄存器参�? */
typedef struct
{
    uint8_t gyroFsr;	/*!< REG_GYROFSR寄存器的�? */
	uint8_t accFsr;
	uint8_t gyroBW;
	uint8_t accBW;
} regValue_t;






/* 模块主动上传的数�? */
extern attitude_t		attitude;		/*!< 姿态角 */
extern quaternion_t		quaternion;
extern gyroAcc_t 		gyroAccData;
extern ioStatus_t		iostatus;

/* 模块寄存器参数�? */
extern regValue_t  	imu901Param;

/* 串口接收解析成功的数据包 */
extern atkp_t 			rxPacket;







void imu901_init(void);

uint8_t imu901_unpack(uint8_t ch);
void atkpParsing(atkp_t *packet);



typedef struct{
    UART_HandleTypeDef * imu_uart;
    volatile float* yaw_ptr;
    float target_angle;
    float init_angle;
    bool switch_; 
    float (*get_angle)(void);
}ATK_IMU_t;

extern ATK_IMU_t  imu;

void IMU_IRQ(void);
void ATK_IMU_Init(void);
void Set_InitYaw(int target);
float Get_Yaw(void);
float angle_limit(float  angle);
void Set_IMUStatus(int status);
int Get_IMUStatus(void);
#endif


 

