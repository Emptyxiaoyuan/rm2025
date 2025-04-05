/**
 * @file chassis_power_control.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
 *        JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
 *        (包括max_vx_speed, min_vx_speed)
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "chassis_power_control.h"
#include "user_c.h"
#include "referee.h"

uint16_t chassis_power_max = 0;


#define ALERT_THRESHOLD		 30.0f
#define WARNING_POWER_BUFF 50.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 4000.0f
#define POWER_TOTAL_CURRENT_LIMIT 4500.0f

/**
 * @brief          限制功率，主要限制电机电流
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
uint8_t robot_id;
void chassis_power_control(chassis_move_t *chassis_power_control)
{

	uint16_t max_power_limit = 20;
	fp32 chassis_max_power = 0;
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;

	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	fp32 a = 1.23e-07;						 // k1
	fp32 k2 = 1.453e-07;					 // k2
	fp32 constant = 4.081f;
	
	get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
	PID_calc(&chassis_power_control->buffer_pid,chassis_power_buffer,60);
	get_chassis_power_max(&max_power_limit);
	
	input_power = max_power_limit - chassis_power_control->buffer_pid.out;
	chassis_max_power = input_power;
	
	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

		if (initial_give_power < 0) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		fp32 power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}

			fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_power_control->motor_speed_pid[i].out = 16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->motor_speed_pid[i].out = -16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
		}
	}

}

//#include "chassis_power_control.h"
//#include "user_c.h"
//#include "referee.h"

//uint16_t chassis_power_max = 0;


//#define ALERT_THRESHOLD		 30.0f
//#define WARNING_POWER_BUFF 50.0f

//#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
//#define BUFFER_TOTAL_CURRENT_LIMIT 4000.0f
//#define POWER_TOTAL_CURRENT_LIMIT 4500.0f

///**
// * @brief          限制功率，主要限制电机电流
// * @param[in]      chassis_power_control: 底盘数据
// * @retval         none
// */
//uint8_t robot_id;
//void chassis_power_control(chassis_move_t *chassis_power_control)
//{

//	uint16_t max_power_limit = 40;
//	fp32 chassis_max_power = 0;
//	float input_power = 0;		 // input power from battery (referee system)
//	float initial_give_power[4]; // initial power from PID calculation
//	float initial_total_power = 0;
//	fp32 scaled_give_power[4];

//	fp32 chassis_power = 0.0f;
//	fp32 chassis_power_buffer = 0.0f;

//	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
//	fp32 a = 1.23e-07;						 // k1
//	fp32 k2 = 1.453e-07;					 // k2
//	fp32 constant = 4.081f;
//	
//	get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
//	PID_calc(&chassis_power_control->buffer_pid,chassis_power_buffer,40);
//	get_chassis_power_max(&max_power_limit);
//	
//	input_power = max_power_limit - chassis_power_control->buffer_pid.out;
//	chassis_max_power = input_power;
//	
//	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
//	{
//		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

//		if (initial_give_power < 0) // negative power not included (transitory)
//			continue;
//		initial_total_power += initial_give_power[i];
//	}

//	if (initial_total_power > chassis_max_power) // determine if larger than max power
//	{
//		fp32 power_scale = chassis_max_power / initial_total_power;
//		for (uint8_t i = 0; i < 4; i++)
//		{
//			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
//			if (scaled_give_power[i] < 0)
//			{
//				continue;
//			}

//			fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
//			fp32 c = k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

//			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
//			{
//				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//				if (temp > 16000)
//				{
//					chassis_power_control->motor_speed_pid[i].out = 16000;
//				}
//				else
//					chassis_power_control->motor_speed_pid[i].out = temp;
//			}
//			else
//			{
//				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//				if (temp < -16000)
//				{
//					chassis_power_control->motor_speed_pid[i].out = -16000;
//				}
//				else
//					chassis_power_control->motor_speed_pid[i].out = temp;
//			}
//		}
//	}

//}
