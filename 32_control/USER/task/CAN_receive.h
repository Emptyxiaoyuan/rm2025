/**
 * @file CAN_receive.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN数据发送 硬件ID*/
typedef enum
{
  //底盘ID
  CAN_CHASSIS_ALL_ID = 0x200,
  //
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,
	
	//摩擦轮和拨弹电机
	CAN_Friction_ALL_ID = 0x1FF,
	
	CAN_Friction_3508_Left = 0x206,
  CAN_Friction_3508_Right = 0x205,
  CAN_TRIGGER_MOTOR_ID = 0x207,
	
	//云台电机
  CAN_GIMBAL_ALL_ID = 0x2FF,
	
	CAN_YAW_MOTOR_ID = 0x209,
  CAN_PIT_MOTOR_ID = 0x20A,
	
	
	
	
	

} can_msg_id_e;

// 电机数据结构体
typedef struct
{
  uint16_t ecd;						//机械角度
  int16_t speed_rpm;			//转速单位：rpm
  int16_t given_current;	//实际电流值
  uint8_t temperate;			//温度
  int16_t last_ecd;				//上次机械角度
} motor_measure_t;

/**
 * @brief          发送电机控制电流(0x209,0x20A,0x20B,NULL)
 * @param[in]      yaw: (0x209) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x20A) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      rev: (0x20B) 保留，电机控制电流
 * @retval         none
 */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev);

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
extern void CAN_cmd_chassis_reset_ID(void);

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      friction_left: (0x205) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      friction_right: (0x206) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
extern void CAN_cmd_Friction(int16_t left,int16_t right,int16_t shoot,int16_t rev);

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
/**
 * @brief          返回摩擦轮左侧电机 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_friction_left_motor_measure_point(void);
/**
 * @brief          返回摩擦轮右侧电机 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_friction_rigth_motor_measure_point(void);




#endif
