/**
 * @file usb_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief usb任务
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "usb_task.h"
#include "user_c.h"

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "user_crc32.h"
#include "detect_task.h"
// #include "voltage_task.h"

//用于存储最后一帧的接收内容
uint8_t usb_rxbuffer[USB_RXLENGTH];



static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;

extern shoot_control_t shoot_control; // 射击数据
extern gimbal_control_t gimbal_control;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

VcpRx_t USB_Temp={
	.rxlen = 0,
	.flag = 0
};

SendPacket USB_RxDataNode;

pid_type_def Input_radians;

fp32 pid[3]={0.1f,0.0f,0.0f};

uint8_t data[6]={0x01,0x02,0x03,0x04,0xd4,0x5e};


/**
 * @brief USB-task;usb数据传输 串口
 *
 * @param argument
 */
void usb_task(void const *argument)
{
  float Get_All_Value[3];
	static const fp32 Radian_PID[3]={1,0,RADIAN_SPEED_PID_KD};
	
  MX_USB_DEVICE_Init();
	PID_init(&Input_radians,PID_POSITION,Radian_PID,RADIAN_SPEED_PID_MAX_OUT,RADIAN_SPEED_PID_MAX_IOUT);
//	PID_init(&Input_radians,PID_POSITION,pid,RADIAN_SPEED_PID_MAX_OUT,RADIAN_SPEED_PID_MAX_IOUT);
	Kalman_Init();
	
	error_list_usb_local = get_error_list_point();

  while (1)
  {
		osDelay(5);
		usb_rxdata_printf();

//		osDelay(1);
//		usb_printf("%0.2f,%0.2f,%0.2f,%0.2f\r\n",shoot_control.motor_fric[1].speed,shoot_control.motor_fric[1].speed_set,shoot_control.motor_fric[0].speed,shoot_control.motor_fric[0].speed_set);
		usb_printf("%0.2f,%0.2f\n",USB_RxDataNode.radion,USB_RxDataNode.yaw);
		
//    osDelay(1000);
//    usb_printf(
//        "******************************\r\n\
//voltage percentage:%d%% \r\n\
//DBUS:%s\r\n\
//chassis motor1:%s\r\n\
//chassis motor2:%s\r\n\
//chassis motor3:%s\r\n\
//chassis motor4:%s\r\n\
//yaw motor:%s\r\n\
//pitch motor:%s\r\n\
//trigger motor:%s\r\n\
//gyro sensor:%s\r\n\
//accel sensor:%s\r\n\
//mag sensor:%s\r\n\
//referee usart:%s\r\n\
//******************************\r\n",
//        get_battery_percentage(),
//        status[error_list_usb_local[DBUS_TOE].error_exist],
//        status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//        status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//        status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//        status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//        status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//        status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//        status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//        status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//        status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//        status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//        status[error_list_usb_local[REFEREE_TOE].error_exist]);
  }
}
/**
 * @brief usb发送数据 (重定义printf)
 *
 * @param fmt
 * @param ...
 */
void usb_printf(const char *fmt, ...)
{
  static va_list ap;
  uint16_t len = 0;

  va_start(ap, fmt);

  len = vsprintf((char *)usb_buf, fmt, ap);

  va_end(ap);

  CDC_Transmit_FS(usb_buf, len);
}


/**
 * @brief usb接收数据
 *
 * @param uint8_t* Buf, uint32_t *Len
 * @param ...
 */
void usb_rxdata_printf(void)
{
	uint16_t crc_data=0;
	uint16_t get_crcdata=0;
	uint32_t len=0;
	uint32_t yaw_data=0;
	uint32_t distance=0;
	uint8_t i=0;
	if(USB_Temp.flag)
	{
		crc_data = PY_CRC_16_T8_USB_i(usb_rxbuffer,USB_Temp.last_rxlen-2);
		get_crcdata = usb_rxbuffer[USB_Temp.last_rxlen-2]<<8 | usb_rxbuffer[USB_Temp.last_rxlen-1];
		if(crc_data == get_crcdata)		//CRC校验成功
		{
//			PID_init(&Input_radians,PID_POSITION,pid,RADIAN_SPEED_PID_MAX_OUT,RADIAN_SPEED_PID_MAX_IOUT);
			
			USB_RxDataNode.header = usb_rxbuffer[len++];
			USB_RxDataNode.reserved = (usb_rxbuffer[len]&0x80)>>7;
			USB_RxDataNode.armors_num = (usb_rxbuffer[len]&0x70)>>3;
			USB_RxDataNode.id = (usb_rxbuffer[len]&0x0e)>>1;
			USB_RxDataNode.tracking = usb_rxbuffer[len++]&0x01;
			for(i=0;i<4;i++){
				yaw_data|=usb_rxbuffer[len++]<<i*8;
			}
			USB_RxDataNode.u32_yaw = yaw_data;
			
			USB_RxDataNode.yaw=KalmanFilter(&kfp,USB_RxDataNode.yaw);
//			if(USB_RxDataNode.yaw <= 0.02f && USB_RxDataNode.yaw)
//			{
//				USB_RxDataNode.yaw=0;
//			}
			USB_RxDataNode.radion = -PID_calc(&Input_radians,USB_RxDataNode.yaw,0);
			if(USB_RxDataNode.radion > 0.2f){
				USB_RxDataNode.radion *= 0.2;
			}else if(USB_RxDataNode.radion <= 0.2f && USB_RxDataNode.radion > 0.1f){
				USB_RxDataNode.radion *=  0.1;
			}else if(USB_RxDataNode.radion <= 0.1f && USB_RxDataNode.radion > 0.05f){
				USB_RxDataNode.radion *=  0.05;
			}else if(USB_RxDataNode.radion <= 0.05f){
				USB_RxDataNode.radion *=  0.02;
			}
			
			
			for(i=0;i<4;i++){
				distance|=usb_rxbuffer[len++]<<i*8;
			}
			USB_RxDataNode.u32_distance = distance;
		}
		
		//数据校验成功或失败均在完成一次接收处理后，对数组内容清空
		USB_Temp.flag = 0;
		USB_Temp.last_rxlen = 0;
		
	}
}

//CRC16-USB校验方式
uint16_t PY_CRC_16_T8_USB_i(uint8_t *di, uint32_t len)
{
	uint16_t crc_poly = 0xA001; //Bit sequence inversion of 0x8005
	uint16_t data_t = 0xFFFF; //CRC register

    for(uint32_t i = 0; i < len; i++)
    {
    	data_t ^= di[i]; //8-bit data

        for (uint8_t j = 0; j < 8; j++)
        {
            if (data_t & 0x0001)
            	data_t = (data_t >> 1) ^ crc_poly;
            else
            	data_t >>= 1;
        }
    }
    return data_t ^ 0xFFFF;
}


Kalman kfp;


void Kalman_Init()
{
	kfp.Last_P = 0.5;			
	kfp.Now_P = 0;		
	kfp.out = 0;			
	kfp.Kg = 0;		
	kfp.Q = 0.2;
	kfp.R = 0.01;
}

/**
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman *kfp,float input)
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}




