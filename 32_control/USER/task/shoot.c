/**
 * @file shoot.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 射击管理函数
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */
 
 
/**
 *	更新摩擦轮控制方式，改为CAN驱动，电机ID为
 */

#include "shoot.h"
#include "user_c.h"
#include "referee.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) // 摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) // 摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         // 关闭两个摩擦轮

#define shoot_laser_on() laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off() // 激光关闭宏定义

// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
 * @brief
 *
 */
static void U_shoot_set_mode(void);
/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

shoot_control_t shoot_control; // 射击数据

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
  static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
  shoot_control.shoot_mode = SHOOT_STOP;
  // 遥控器指针
  shoot_control.shoot_rc = get_remote_control_point();
  // 拨弹电机指针
  shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
  // 初始化PID
  PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
  
	//初始化摩擦轮电机
	static const fp32 Friction_left_speed_pid[3] = {FRICTION_MOTOR_LEFT_SPEED_PID_KP,FRICTION_MOTOR_LEFT_SPEED_PID_KI,FRICTION_MOTOR_LEFT_SPEED_PID_KD};
	static const fp32 Friction_right_speed_pid[3] = {FRICTION_MOTOR_RIGHT_SPEED_PID_KP,FRICTION_MOTOR_RIGHT_SPEED_PID_KI,FRICTION_MOTOR_RIGHT_SPEED_PID_KD};
	//左侧电机指针
	shoot_control.motor_fric[0].fric_motor_measure = get_friction_rigth_motor_measure_point();
	//右侧电机指针
	shoot_control.motor_fric[1].fric_motor_measure = get_friction_left_motor_measure_point();
	//初始化两个电机的PID
	PID_init(&shoot_control.fric_motor_pid[0],PID_POSITION,Friction_left_speed_pid,FRICTION_MOTOR_LEFT_SPEED_PID_MAX_OUT,FRICTION_MOTOR_LEFT_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.fric_motor_pid[1],PID_POSITION,Friction_right_speed_pid,FRICTION_MOTOR_RIGHT_SPEED_PID_MAX_OUT,FRICTION_MOTOR_RIGHT_SPEED_PID_MAX_IOUT);
	
	
	// 更新数据
  shoot_feedback_update();
	
	//设置摩擦轮最大速度和最小速度（最小速度不可更改，可以更改最大速度，并且保证不超过设置值）
	shoot_control.fric_speed_max[0] = -FRIC_SPEED_MAX;
	shoot_control.fric_speed_min[0] = -FRIC_SPEED_MIN;
	
	shoot_control.fric_speed_max[1] = FRIC_SPEED_MAX;
	shoot_control.fric_speed_min[1] = FRIC_SPEED_MIN;
	

  // 初始化 射击控制结构体
  shoot_control.fric_pwm1 = FRIC_OFF;
  shoot_control.fric_pwm2 = FRIC_OFF;
  shoot_control.ecd_count = 0;
  shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
  shoot_control.given_current = 0;
  shoot_control.move_flag = 0;
  shoot_control.set_angle = shoot_control.angle;
  shoot_control.speed = 0.0f;
  shoot_control.speed_set = 0.0f;
  shoot_control.key_time = 0;
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
int16_t shoot_control_loop(void)
{
	//用于保存按钮是否断开
	static uint8_t rc_flag=0;
	
  shoot_set_mode(); // 设置状态机
  // U_shoot_set_mode();      // user
  shoot_feedback_update(); // 更新数据
	
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    // 设置拨弹轮的速度
    shoot_control.speed_set = 0.0f;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
  {
    // 设置拨弹轮的速度
    shoot_control.speed_set = 0.0f;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
  {
    if (shoot_control.key == SWITCH_TRIGGER_OFF)
    {
      // 设置拨弹轮的拨动速度,并开启堵转反转处理
      shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;

      trigger_motor_turn_back();
    }
    else
    {
      shoot_control.trigger_speed_set = 0.0f;
      shoot_control.speed_set = 0.0f;
    }
    shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
    shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    // 设置拨弹轮的速度
    shoot_control.speed_set = 0.0f;
  }
  else if (shoot_control.shoot_mode == SHOOT_BULLET)
  {
    shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    shoot_bullet_control();
  }
  // 继续射击模式
  else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
  {
    // 设置拨弹轮的拨动速度,并开启堵转反转处理
    shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
    // 堵转倒转控制
    trigger_motor_turn_back();
  }
  // 射击模式 射击完毕
  else if (shoot_control.shoot_mode == SHOOT_DONE)
  {
    shoot_control.speed_set = 0.0f;
  }
  // 射击模式 = 停止射击
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    // 关闭激光
    shoot_laser_off();
    shoot_control.given_current = 0.0f;
		
		//关闭摩擦轮
		shoot_control.motor_fric[0].speed_set =0.0f;
		shoot_control.motor_fric[1].speed_set =0.0f;
		
		shoot_control.fric_speed_max[0] = -FRIC_SPEED_MAX;
		shoot_control.fric_speed_max[1] = FRIC_SPEED_MAX;
		
  }
  else
  {
    // 激光开启
    shoot_laser_on();
    // 计算拨弹轮电机PID
    PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
    if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
    {
      shoot_control.given_current = 0;
    }
		else if(shoot_control.shoot_mode > SHOOT_READY_BULLET  && shoot_control.speed_set == 0)
		{
			shoot_control.given_current = 0;
		}
		
		shoot_control.motor_fric[0].speed_set =-shoot_control.fric_speed_max[0];
		shoot_control.motor_fric[1].speed_set =-shoot_control.fric_speed_max[1];
		//此处通过遥控器的左上角旋转按钮实现发射的速度
		if((shoot_control.shoot_rc->rc.ch[4] >= 440 && rc_flag == 0) || (shoot_flag == 0)){
			rc_flag = 1;
			shoot_flag = 2;
			shoot_control.fric_speed_max[0]-=FRIC_ONCE;
			if(shoot_control.fric_speed_max[0] <= -MAX_FRIC_SPEED)
			{
				shoot_control.fric_speed_max[0] = -MAX_FRIC_SPEED;
			}

			shoot_control.fric_speed_max[1] += FRIC_ONCE;
			if(shoot_control.fric_speed_max[1] >= MAX_FRIC_SPEED)
			{
				shoot_control.fric_speed_max[1] = MAX_FRIC_SPEED;
			}
			

		}else if((shoot_control.shoot_rc->rc.ch[4] <= -440 && rc_flag == 0) || (shoot_flag == 1)){
			rc_flag = 1;
			shoot_flag = 2;
			shoot_control.fric_speed_max[0]+=FRIC_ONCE;
			if(shoot_control.fric_speed_max[0] >= -FRIC_SPEED_MIN)
			{
				shoot_control.fric_speed_max[0] = -FRIC_SPEED_MIN;
			}

			shoot_control.fric_speed_max[1] -= FRIC_ONCE;
			if(shoot_control.fric_speed_max[1] <= FRIC_SPEED_MIN)
			{
				shoot_control.fric_speed_max[1] = FRIC_SPEED_MIN;
			}

			
		}else if((shoot_control.shoot_rc->rc.ch[4] == 0 && rc_flag == 1) || (shoot_flag != 2)){
			rc_flag = 0;
			shoot_flag = 2;
		}
		

  }

  
	Friction_Control_Loop(&shoot_control);
	
	if(!(toe_is_error(FRICTION_LEFT_MOTOR_TOE) && toe_is_error(FRICTION_RIGHT_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
	{
		if(toe_is_error(DBUS_TOE))
		{
			CAN_cmd_Friction(0,0,0,0);
		}
		else
		{
			//发送控制电流
			CAN_cmd_Friction(shoot_control.motor_fric[0].give_current,
											 shoot_control.motor_fric[1].give_current,
											 shoot_control.given_current,0);
		}
		printf("%f,%f,%f,%f\n",shoot_control.motor_fric[0].speed,shoot_control.motor_fric[0].speed_set,-shoot_control.motor_fric[1].speed,-shoot_control.motor_fric[1].speed_set);
	}
	
	
	
	return shoot_control.given_current;
}
/**
 * @brief 射击状态机设置
 *
 */
void U_shoot_set_mode(void)
{
  const RC_ctrl_t *test_Shoot_RC;
  // 获取遥控器数据
  test_Shoot_RC = get_remote_control_point();
  if (switch_is_up(test_Shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]))
  {
    // 射击进入摩擦轮准备
    shoot_control.shoot_mode = SHOOT_READY_FRIC;
    // 判断左上角拨轮
    if (test_Shoot_RC->rc.ch[4] >= 10 && test_Shoot_RC->rc.ch[4] <= 600)
    {
      // 备弹
      shoot_control.shoot_mode = SHOOT_READY_BULLET;
      // 射击状态
      shoot_control.shoot_mode = SHOOT_BULLET;
    }
    // 低速射击
    else if (test_Shoot_RC->rc.ch[4] == 660)
    {
      // 提高摩擦轮转速
      shoot_control.fric1_ramp.max_value = FRIC_UP;
      shoot_control.fric2_ramp.max_value = FRIC_UP;
    }
    // 高速射击
    else if (test_Shoot_RC->rc.ch[4] == -660)
    {
      // 减小摩擦轮转速
      shoot_control.fric1_ramp.max_value = FRIC_DOWN;
      shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }
  }
  else
  {
    shoot_control.shoot_mode = SHOOT_STOP;
  }

  // 如果云台状态是 无力状态，就关闭射击
  if (gimbal_cmd_to_shoot_stop())
  {
    shoot_control.shoot_mode = SHOOT_STOP;
  }
}

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
  static int8_t last_s = RC_SW_UP;

  // 上拨判断， 一次开启，再次关闭
  if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
  {
    shoot_control.shoot_mode = SHOOT_READY_FRIC;
  }
  else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
  {
    shoot_control.shoot_mode = SHOOT_STOP;
  }

  // 处于中档， 可以使用键盘开启摩擦轮
  if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
  {
    shoot_control.shoot_mode = SHOOT_READY_FRIC;
  }
  // 处于中档， 可以使用键盘关闭摩擦轮
  else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
  {
    shoot_control.shoot_mode = SHOOT_STOP;
  }
	
//	//进入射击模式
//	if((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
//	{
//		//只有当前状态摩擦轮在发弹就绪时，才可以进入射击准备模式，其他时候保持射击模式就绪
//		if(shoot_control.shoot_mode > SHOOT_STOP)
//		{
//			shoot_control.shoot_mode = SHOOT_READY;
//		}
//	}else if(switch_is_down(last_s) && (shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL] != 2))
//	{
//		if(shoot_control.shoot_mode > SHOOT_STOP)
//		{
//			shoot_control.shoot_mode = SHOOT_READY_FRIC;
//		}
//	}
	
	
  if (shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
  {
    shoot_control.shoot_mode = SHOOT_READY_BULLET;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
  {
    shoot_control.shoot_mode = SHOOT_READY;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
  {
    shoot_control.shoot_mode = SHOOT_READY_BULLET;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    // 下拨一次或者鼠标按下一次，进入射击状态
    if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
    {
      shoot_control.shoot_mode = SHOOT_BULLET;
    }
  }
  else if (shoot_control.shoot_mode == SHOOT_DONE)
  {
    if (shoot_control.key == SWITCH_TRIGGER_OFF)
    {
      shoot_control.key_time++;
      if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
      {
        shoot_control.key_time = 0;
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
      }
    }
    else
    {
      shoot_control.key_time = 0;
      shoot_control.shoot_mode = SHOOT_BULLET;
    }
  }

  if (shoot_control.shoot_mode > SHOOT_READY_FRIC)
  {
    // 鼠标长按一直进入射击状态 保持连发
    if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
    {
      shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
      shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
  }

  get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);

  /**TODO:*/
  // 步兵枪口热量上限,240
  if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
	{
		if (shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_READY_BULLET;
		}
	}
  /*TODO:*/
  

  // 如果云台状态是 无力状态，就关闭射击
  if (gimbal_cmd_to_shoot_stop())
  {
    shoot_control.shoot_mode = SHOOT_STOP;
  }

  last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{

  // 二阶低通滤波
  shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	
	//摩擦轮电机滤波
	shoot_control.motor_fric[1].speed = shoot_control.motor_fric[1].fric_motor_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	shoot_control.motor_fric[0].speed = shoot_control.motor_fric[0].fric_motor_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	
	
  // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    shoot_control.ecd_count--;
  }
  else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    shoot_control.ecd_count++;
  }

  if (shoot_control.ecd_count == FULL_COUNT)
  {
    shoot_control.ecd_count = -(FULL_COUNT - 1);
  }
  else if (shoot_control.ecd_count == -FULL_COUNT)
  {
    shoot_control.ecd_count = FULL_COUNT - 1;
  }

  // 计算输出轴角度
  shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
  // 微动开关
  shoot_control.key = BUTTEN_TRIG_PIN;

  // 鼠标按键
  shoot_control.last_press_l = shoot_control.press_l;
  shoot_control.last_press_r = shoot_control.press_r;
  shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
  shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

  // 长按计时
  // 鼠标左键按下
  if (shoot_control.press_l)
  {
    if (shoot_control.press_l_time < PRESS_LONG_TIME)
    {
      shoot_control.press_l_time++;
    }
  }
  else
  {
    shoot_control.press_l_time = 0;
  }
  // 鼠标右键按下
  if (shoot_control.press_r)
  {
    if (shoot_control.press_r_time < PRESS_LONG_TIME)
    {
      shoot_control.press_r_time++;
    }
  }
  else
  {
    shoot_control.press_r_time = 0;
  }

  //射击开关下档时间计时
  // 遥控器左侧拨到下档,连续发射弹丸 TODO:
   if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
   {
     if (shoot_control.rc_s_time < RC_S_LONG_TIME)
     {
       shoot_control.rc_s_time++;
     }
   }
   else
   {
     shoot_control.rc_s_time = 0;
   }

//  // 鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
//  static uint16_t up_time = 0;
//	static uint8_t up_flag=0;
//  if (shoot_control.press_r)
//  {
//		up_flag = 1;
//    up_time = UP_ADD_TIME;
//  }
//  // 任务执行过快 此处时间使摩擦轮响应一段时间
//	if(up_flag == 1){
//		if (up_time > 0)
//		{
//			shoot_control.fric_speed_max[0] = FRIC_SPEED_MAX+0.5f;
//			shoot_control.fric_speed_max[1] = FRIC_SPEED_MAX+0.5f;
//			up_time--;
//		}
//		else
//		{
//			up_flag = 0;
//			shoot_control.fric_speed_max[0] = FRIC_SPEED_MAX+0.5f;
//			shoot_control.fric_speed_max[1] = FRIC_SPEED_MAX+0.5f;
//		}
//	}
  
}
/**
 * @brief 堵转倒转处理
 * 700ms 确定其堵转
 */
static void trigger_motor_turn_back(void)
{
  if (shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.speed_set = shoot_control.trigger_speed_set;
  }
  else
  {
    shoot_control.speed_set = -shoot_control.trigger_speed_set;
  }

  if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.block_time++;
    shoot_control.reverse_time = 0;
  }
  else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
  {
    shoot_control.reverse_time++;
  }
  else
  {
    shoot_control.block_time = 0;
  }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
  // 每次拨动 1/4PI的角度
  if (shoot_control.move_flag == 0)
  {
    shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
    shoot_control.move_flag = 1;
  }
  // 检查是否有弹丸经过
  if (shoot_control.key == SWITCH_TRIGGER_OFF)
  {
    shoot_control.shoot_mode = SHOOT_DONE;
  }
  // 到达角度判断
  if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
  {
    // 没到达一直设置旋转速度
    shoot_control.trigger_speed_set = TRIGGER_SPEED;
    trigger_motor_turn_back();
  }
  else
  {
    shoot_control.move_flag = 0;
  }
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     Friction_Control_Loop:"shoot_control_t"变量指针.
 * @retval         none
 */
void Friction_Control_Loop(shoot_control_t *fric_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
	uint8_t i = 0;
	
	//计算摩擦轮并控制最大速度，同时限制最大速度
	for(i =0 ;i<2;i++)
	{
		temp = fabs(fric_move_control_loop->motor_fric[i].speed_set); 
		if(max_vector < temp)
		{
			max_vector = temp;
		}
	}
	
	//速度限制
	if(max_vector > MAX_FRIC_SPEED)
	{
		vector_rate = MAX_FRIC_SPEED / max_vector;
		for(i=0;i<2;i++)
		{
			fric_move_control_loop->motor_fric[i].speed_set *= vector_rate;
		}
	}
	
	//计算PID
	for(i=0;i<2;i++)
	{
		PID_calc(&fric_move_control_loop->fric_motor_pid[i],fric_move_control_loop->motor_fric[i].speed,fric_move_control_loop->motor_fric[i].speed_set);
	}
	
	//赋值电流值
	for(i=0;i<2;i++)
	{
		fric_move_control_loop->motor_fric[i].give_current = (int16_t)(fric_move_control_loop->fric_motor_pid[i].out);
	}
}

