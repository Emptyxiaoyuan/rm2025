#ifndef __USB_TASK_H
#define __USB_TASK_H

#include "usb_task.h"
#include "user_c.h"

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


#define USB_RXLENGTH		256

void usb_task(void const *argument);
void usb_printf(const char *fmt, ...);
void usb_rxdata_printf(void);

uint16_t PY_CRC_16_T8_USB_i(uint8_t *di, uint32_t len);

extern uint8_t usb_rxbuffer[256];


typedef struct{
	uint32_t rxlen;
	uint32_t last_rxlen;
	uint32_t flag;
}VcpRx_t;


extern VcpRx_t USB_Temp;


//定义接收NUC数据格式
typedef struct{
	uint8_t header;		//帧头
	
	uint8_t tracking : 1;		//检测是否识别成功
	uint8_t id : 3;					//装甲板ID（未识别装甲板ID为0）
  uint8_t armors_num : 3;	//装甲板类型（步兵，工程，哨兵）
  uint8_t reserved : 1;		//保留内容
							// 上面占一个字节，id表示识别到的数字，armors_num表示装甲板类型（2-balance 3-outpost 4-normal）
	union{
		uint32_t u32_yaw;
		float yaw; 				// 目标在摄像头视角内左右方向的偏角，左正右负
	};
	union{
		uint32_t u32_distance;
		float distance; 				// 目标与摄像头距离(误差较大)
	};
	fp32 radion;
}SendPacket;

extern SendPacket USB_RxDataNode;


// radian 角度环 PID参数以及 PID最大输出，积分输出
#define RADIAN_SPEED_PID_KP 1.5f
#define RADIAN_SPEED_PID_KI 0.0f
#define RADIAN_SPEED_PID_KD 0.0f
#define RADIAN_SPEED_PID_MAX_OUT 10.0f
#define RADIAN_SPEED_PID_MAX_IOUT 100.0f



typedef unsigned char Kalman_U8;
typedef unsigned short int Kalman_U16;
typedef unsigned long int Kalman_U32;
typedef float Kalman_F8;
 
typedef struct 
{
    float Last_P;//上次估算协方差 不可以为0 ! ! ! ! ! 
    float Now_P;//当前估算协方差
    float out;//卡尔曼滤波器输出
    float Kg;//卡尔曼增益
    float Q;//过程噪声协方差
    float R;//观测噪声协方差
}Kalman;

void Kalman_Init(void);
float KalmanFilter(Kalman *kfp,float input);

extern Kalman kfp;






#endif /*__USB_TASK_H*/
