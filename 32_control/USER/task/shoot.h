/**
 * @file shoot.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 射击任务
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote.h"
#include "user_lib.h"

// 射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL 1
// 云台模式使用的开关通道

#define SHOOT_CONTROL_TIME GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE 100.0f

// 射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

// 射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 10
// 鼠标长按判断
#define PRESS_LONG_TIME 100
// 遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
//#define RC_S_LONG_TIME 2000
#define RC_S_LONG_TIME 100
// 摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
// 电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
// 电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
// 电机编码器角度 变化成 相对夹角的比例
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
// 拨弹速度
//#define TRIGGER_SPEED 4.0f
//#define CONTINUE_TRIGGER_SPEED 8.0f
//#define READY_TRIGGER_SPEED 4.0f
#define TRIGGER_SPEED -4.0f
#define CONTINUE_TRIGGER_SPEED -8.0f
#define READY_TRIGGER_SPEED -4.0f

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

// 卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED 1.0f

#define BLOCK_TIME 700
// 反转时间
#define REVERSE_TIME 600
// 反转速度限制
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_FOUR 0.78539816339744830961566084581988f
#define PI_TEN 0.314f

// 拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 800.0f
#define TRIGGER_ANGLE_PID_KI 0.5f
#define TRIGGER_ANGLE_PID_KD 1.0f

#define TRIGGER_BULLET_PID_MAX_OUT 10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT 10000.0f
#define TRIGGER_READY_PID_MAX_IOUT 7000.0f

#define SHOOT_HEAT_REMAIN_VALUE 40

//摩擦轮左侧电机PID
#define FRICTION_MOTOR_LEFT_SPEED_PID_KP 15000.0f
#define FRICTION_MOTOR_LEFT_SPEED_PID_KI 10.0f
#define FRICTION_MOTOR_LEFT_SPEED_PID_KD 0.0f
#define FRICTION_MOTOR_LEFT_SPEED_PID_MAX_OUT 16000.0f
#define FRICTION_MOTOR_LEFT_SPEED_PID_MAX_IOUT 2000.0f

//摩擦轮右侧电机PID
#define FRICTION_MOTOR_RIGHT_SPEED_PID_KP 15000.0f
#define FRICTION_MOTOR_RIGHT_SPEED_PID_KI 10.0f
#define FRICTION_MOTOR_RIGHT_SPEED_PID_KD 0.0f
#define FRICTION_MOTOR_RIGHT_SPEED_PID_MAX_OUT 16000.0f
#define FRICTION_MOTOR_RIGHT_SPEED_PID_MAX_IOUT 2000.0f



typedef enum
{
  // 射击停止
  SHOOT_STOP = 0,
  // 射击 准备 击发
  SHOOT_READY_FRIC,
  // 射击 准备 子弹
  SHOOT_READY_BULLET,
  // 射击准备
  SHOOT_READY,
  // 射击 子弹
  SHOOT_BULLET,
  // 射击 继续 子弹
  SHOOT_CONTINUE_BULLET,
  // 射击完成
  SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
	//摩擦轮电机
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
}fric_motor_t;

typedef struct
{
  shoot_mode_e shoot_mode;
  const RC_ctrl_t *shoot_rc;
  //获取电机反馈数据
	//拨弹电机
	const motor_measure_t *shoot_motor_measure;
	
	//设置摩擦轮电机数据
	fric_motor_t motor_fric[2];
	
	
  // 摩擦轮1
  ramp_function_source_t fric1_ramp;
  uint16_t fric_pwm1;
  // 摩擦轮2
  ramp_function_source_t fric2_ramp;
  uint16_t fric_pwm2;
  // 拨弹轮 pid
  pid_type_def trigger_motor_pid;
	//摩擦轮 pid
	pid_type_def fric_motor_pid[2];
	
	//摩擦轮最大速度
	fp32 fric_speed_max[2];
	fp32 fric_speed_min[2];
	
	
	
  // 拨弹轮速度
  fp32 trigger_speed_set;
  fp32 speed;
  fp32 speed_set;
  fp32 angle;
  fp32 set_angle;
  int16_t given_current;
  int8_t ecd_count;

  bool_t press_l;
  bool_t press_r;
  bool_t last_press_l;
  bool_t last_press_r;
  uint16_t press_l_time;
  uint16_t press_r_time;
  uint16_t rc_s_time;

  uint16_t block_time;
  uint16_t reverse_time;
  bool_t move_flag;

  bool_t key;
  uint8_t key_time;

  uint16_t heat_limit;
  uint16_t heat;
} shoot_control_t;

// 由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
// 射击循环
extern int16_t shoot_control_loop(void);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制，计算摩擦轮电流
 * @param[out]     Friction_Control_Loop:"shoot_control_t"变量指针.
 * @retval         none
 */
extern void Friction_Control_Loop(shoot_control_t *fric_move_control_loop);

#endif
