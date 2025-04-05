#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
  RED_HERO = 1,
  RED_ENGINEER = 2,
  RED_STANDARD_1 = 3,
  RED_STANDARD_2 = 4,
  RED_STANDARD_3 = 5,
  RED_AERIAL = 6,
  RED_SENTRY = 7,
  BLUE_HERO = 11,
  BLUE_ENGINEER = 12,
  BLUE_STANDARD_1 = 13,
  BLUE_STANDARD_2 = 14,
  BLUE_STANDARD_3 = 15,
  BLUE_AERIAL = 16,
  BLUE_SENTRY = 17,
} robot_id_t;
typedef enum
{
  PROGRESS_UNSTART = 0,
  PROGRESS_PREPARE = 1,
  PROGRESS_SELFCHECK = 2,
  PROGRESS_5sCOUNTDOWN = 3,
  PROGRESS_BATTLE = 4,
  PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0001
{
  uint8_t game_type : 4;						//比赛类型
  uint8_t game_progress : 4;				//当前比赛阶段
  uint16_t stage_remain_time;				//当前阶段剩余时间
	uint64_t SyncTimeStamp;					//UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} ext_game_state_t;						

typedef __packed struct // 0002
{
  uint8_t winner;			//判断胜负（0：平局，1：红方胜利，2：蓝方胜利）
} ext_game_result_t;
typedef __packed struct
{
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __packed struct // 0101
{
  uint32_t event_type;
} ext_event_data_t;

typedef __packed struct // 0x0102
{
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct // 0x0103
{
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
  uint8_t level;
  uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct // 0x0201
{
  uint8_t robot_id; // 本机器人ID
  uint8_t robot_level;		// 机器人等级
  uint16_t current_HP;     // 机器人当前血量
  uint16_t maximum_HP;        // 机器人血条上限
  uint16_t shooter_barrel_cooling_value;  // 冷却值/s
  uint16_t shooter_barrel_heat_limit; // 热量上限
  uint16_t chassis_power_limit;		// 机器人底盘功率上限
  uint8_t power_management_gimbal_output : 1;		// 云台口电源输出情况(0:OFF,1:ON)
  uint8_t power_management_chassis_output : 1;	// 底盘口电源输出情况(0:OFF,1:ON)
  uint8_t power_management_shooter_output : 1;	// 发射口电源输出情况(0:OFF,1:ON)
} ext_game_robot_state_t;

typedef __packed struct // 0x0202
{
  uint16_t chassis_volt;			// 底盘输出电压(单位:mv)
  uint16_t chassis_current;		// 底盘输出电流(单位:mA)
  float chassis_power;				// 底盘功率(单位:W)
  uint16_t chassis_power_buffer;		// 超过限制功率的缓冲能量(单位:J)
  uint16_t shooter_heat0;			// 第一个17mm发射机构的枪口热量
  uint16_t shooter_heat1;			// 第二个17mm发射机构的枪口热量
	uint16_t shooter_42mm_heat;	// 42mm发射机构枪口热量
} ext_power_heat_data_t;

typedef __packed struct // 0x0203
{
  float x;			// 本机器人位置X坐标，单位：M
  float y;			// 本机器人位置Y坐标，单位：M
  float yaw;		// 本机器人测速模块的朝向，单位：度。正北为 0 度
} ext_game_robot_pos_t;

typedef __packed struct // 0x0204
{
  uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct // 0x0205
{
  uint8_t energy_point;
  uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct // 0x0206
{
  uint8_t armor_type : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct // 0x0207
{
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct
{
  uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;
typedef __packed struct // 0x0301
{
  uint16_t send_ID;
  uint16_t receiver_ID;
  uint16_t data_cmd_id;
  uint16_t data_len;
  uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
  float data1;
  float data2;
  float data3;
  uint8_t data4;
} custom_data_t;

typedef __packed struct
{
  uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
  uint8_t data[32];
} ext_download_stream_data_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_chassis_power_max(uint16_t *power);

extern ext_game_robot_state_t robot_state;		// 机器人当前等级，经验，功率上限，热量上限等数据

uint8_t get_robot_state_id(void);


#endif
