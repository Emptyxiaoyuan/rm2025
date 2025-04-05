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


//�������NUC���ݸ�ʽ
typedef struct{
	uint8_t header;		//֡ͷ
	
	uint8_t tracking : 1;		//����Ƿ�ʶ��ɹ�
	uint8_t id : 3;					//װ�װ�ID��δʶ��װ�װ�IDΪ0��
  uint8_t armors_num : 3;	//װ�װ����ͣ����������̣��ڱ���
  uint8_t reserved : 1;		//��������
							// ����ռһ���ֽڣ�id��ʾʶ�𵽵����֣�armors_num��ʾװ�װ����ͣ�2-balance 3-outpost 4-normal��
	union{
		uint32_t u32_yaw;
		float yaw; 				// Ŀ��������ͷ�ӽ������ҷ����ƫ�ǣ������Ҹ�
	};
	union{
		uint32_t u32_distance;
		float distance; 				// Ŀ��������ͷ����(���ϴ�)
	};
	fp32 radion;
}SendPacket;

extern SendPacket USB_RxDataNode;


// radian �ǶȻ� PID�����Լ� PID���������������
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
    float Last_P;//�ϴι���Э���� ������Ϊ0 ! ! ! ! ! 
    float Now_P;//��ǰ����Э����
    float out;//�������˲������
    float Kg;//����������
    float Q;//��������Э����
    float R;//�۲�����Э����
}Kalman;

void Kalman_Init(void);
float KalmanFilter(Kalman *kfp,float input);

extern Kalman kfp;






#endif /*__USB_TASK_H*/
