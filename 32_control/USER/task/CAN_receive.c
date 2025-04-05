/**
 * @file CAN_receive.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "CAN_receive.h"
#include "user_c.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// 电机数据读取
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
/*
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
	4:摩擦轮右侧电机 3508电机; 5:摩擦轮左侧电机 3508电机; 6:拨弹电机 2006电机; 
	7:yaw云台电机 6020电机; 8:pitch云台电机 6020电机*/
static motor_measure_t motor_chassis[9];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static CAN_TxHeaderTypeDef friction_tx_message;
static uint8_t chassis_can_send_data[8];
static uint8_t friction_can_send_data[8];

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	static uint8_t i = 0;
	
  switch (rx_header.StdId)
  {
  case CAN_3508_M1_ID:
  case CAN_3508_M2_ID:
  case CAN_3508_M3_ID:
  case CAN_3508_M4_ID:
  case CAN_Friction_3508_Left:
  case CAN_Friction_3508_Right:
  case CAN_TRIGGER_MOTOR_ID:
  {
    
    // get motor id
    i = rx_header.StdId - CAN_3508_M1_ID;
    get_motor_measure(&motor_chassis[i], rx_data);
    detect_hook(CHASSIS_MOTOR1_TOE + i);
    break;
  }
	case CAN_YAW_MOTOR_ID:
	{
		get_motor_measure(&motor_chassis[7], rx_data);
		detect_hook(YAW_GIMBAL_MOTOR_TOE);
		break;
	}
	case CAN_PIT_MOTOR_ID:
	{
		get_motor_measure(&motor_chassis[8], rx_data);
		detect_hook(PITCH_GIMBAL_MOTOR_TOE);
		break;
	}
	

  default:
  {
    break;
  }
  }
}

/**
 * @brief          发送电机控制电流(0x209,0x20A,0x20B,NULL)
 * @param[in]      yaw: (0x209) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x20A) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      rev: (0x20B) 保留，电机控制电流
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x06;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (rev >> 8);
  gimbal_can_send_data[5] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      friction_left: (0x205) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      friction_right: (0x206) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
void CAN_cmd_Friction(int16_t left,int16_t right,int16_t shoot,int16_t rev)
{
	uint32_t send_mail_box;
  friction_tx_message.StdId = CAN_Friction_ALL_ID;
  friction_tx_message.IDE = CAN_ID_STD;
  friction_tx_message.RTR = CAN_RTR_DATA;
  friction_tx_message.DLC = 0x08;
  friction_can_send_data[0] = left >> 8;
  friction_can_send_data[1] = left;
  friction_can_send_data[2] = right >> 8;
  friction_can_send_data[3] = right;
  friction_can_send_data[4] = shoot >> 8;
  friction_can_send_data[5] = shoot;
  friction_can_send_data[6] = rev >> 8;
  friction_can_send_data[7] = rev;
	
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &friction_tx_message, friction_can_send_data, &send_mail_box);
}



/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[7];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[8];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

/**
 * @brief          返回摩擦轮左侧电机 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_friction_left_motor_measure_point(void)
{
	return &motor_chassis[5];
}

/**
 * @brief          返回摩擦轮右侧电机 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_friction_rigth_motor_measure_point(void)
{
	return &motor_chassis[4];
}

