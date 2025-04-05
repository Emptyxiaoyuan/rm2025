/**
 * @file gimbal_task.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote.h"

// pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 10000.0f
#define PITCH_SPEED_PID_KI 20.0f
#define PITCH_SPEED_PID_KD 3.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 15000.0f

// yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 12500.0f
#define YAW_SPEED_PID_KI 30.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 10000.0f

// pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.4f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

// yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 16.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

// pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

// yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 26.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

// 任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
// yaw,pitch控制通道以及状态开关通道

// 切换左右手 01-23

#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

// 掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
// 掉头云台速度
#define TURN_SPEED 0.04f
// 测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

#define YAW_RC_SEN -0.000005f
#define YAW_RC_SEN_AIMING	-0.000001f

#define PITCH_RC_SEN -0.000006f // 0.005


#define YAW_MOUSE_SEN 0.0001f
#define YAW_MOUSE_SEN_AIMING	0.00005f


#define PITCH_MOUSE_SEN 0.00003f





#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

// 云台电机 系统仍任务延时
#define GIMBAL_CONTROL_TIME 1

// 云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

// 俯仰角翻转
#define PITCH_TURN 0
// 偏航角翻转
#define YAW_TURN 0

// 电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

// 云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f

// 云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED 0.005f

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

// 云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 7000
#define GIMBAL_CALI_MOTOR_PITCH_SET		10000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

// 判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

// 电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

#endif
/**
 * @brief 云台电机模式
 * @param GIMBAL_MOTOR_RAW:原始值控制
 * @param GIMBAL_MOTOR_GYRO:陀螺仪角度控制
 * @param GIMBAL_MOTOR_ENCONDE:编码值角度控制
 *
 */
typedef enum
{
  // 电机原始值控制
  GIMBAL_MOTOR_RAW = 0,
  // 电机陀螺仪角度控制
  GIMBAL_MOTOR_GYRO,
  // 电机编码值角度控制
  GIMBAL_MOTOR_ENCONDE,
} gimbal_motor_mode_e;

/**
 * @brief 云台PID数据结构体
 *
 */
typedef struct
{
  fp32 kp;
  fp32 ki;
  fp32 kd;

  fp32 set;
  fp32 get;
  fp32 err;

  fp32 max_out;
  fp32 max_iout;

  fp32 Pout;
  fp32 Iout;
  fp32 Dout;

  fp32 out;
} gimbal_PID_t;
/**
 * @brief 云台电机数据
 *
 */

typedef struct
{
  // 云台电机数据
  const motor_measure_t *gimbal_motor_measure;

  // 云台PID
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  // 云台电机相对角PID
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  // 云台陀螺仪PID
  pid_type_def gimbal_motor_gyro_pid;
  // 云台控制模式
  gimbal_motor_mode_e gimbal_motor_mode;
  // 上一个云台控制模式
  gimbal_motor_mode_e last_gimbal_motor_mode;
  // 补偿
  uint16_t offset_ecd;
  // 最大相对角
  fp32 max_relative_angle; // rad_
  // 最小相对角
  fp32 min_relative_angle; // rad
  // 相对角
  fp32 relative_angle; // rad
  // 相对角设置
  fp32 relative_angle_set; // rad
  // 绝对角
  fp32 absolute_angle; // rad
  // 绝对角设置
  fp32 absolute_angle_set; // rad
  // 电机陀螺仪
  fp32 motor_gyro; // rad/s
  // 电机陀螺仪设置
  fp32 motor_gyro_set;
  // 电机速度
  fp32 motor_speed;

  fp32 raw_cmd_current;
  // 电流设置
  fp32 current_set;
  // 额定电流
  int16_t given_current;

} gimbal_motor_t;
/**
 * @brief
 *
 */
typedef struct
{
  fp32 max_yaw;
  fp32 min_yaw;
  fp32 max_pitch;
  fp32 min_pitch;
  uint16_t max_yaw_ecd;
  uint16_t min_yaw_ecd;
  uint16_t max_pitch_ecd;
  uint16_t min_pitch_ecd;
  uint8_t step;
} gimbal_step_cali_t;
/**
 * @brief 电机控制结构体
 *
 */
typedef struct
{
  // 遥控器数据
  const RC_ctrl_t *gimbal_rc_ctrl;
  // 云台 欧拉角
  const fp32 *gimbal_INT_angle_point;
  // 云台 角速度
  const fp32 *gimbal_INT_gyro_point;
  // 云台电机偏航角
  gimbal_motor_t gimbal_yaw_motor;
  // 云台电机俯仰角
  gimbal_motor_t gimbal_pitch_motor;
  //
  gimbal_step_cali_t gimbal_cali;

} gimbal_control_t;


//瞄准模式
#define			Manual_mode						0
#define			Self_aiming_mode			1



/**
 * @brief 键位内容结构体
 *
 */
typedef struct
{
	uint8_t aiming_mode:1;		//自瞄：0为关闭，1为开启
	uint8_t aiming_flag:1;		
	
	uint8_t shell_cover:1;		//装弹盖，0为开启，1为关闭
	uint8_t shell_flag:1;
	
	uint8_t color_change:1;		//装甲板颜色变化：0为红色，1为蓝色
	uint8_t color_flag:1;
}gimbal_Aiming_Mode;



/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);
/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);
/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void gimbal_task(void const *pvParameters);
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern gimbal_Aiming_Mode key_content;

#endif
