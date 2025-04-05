/**
 * @file calibrate_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 设备校准
 * @version 0.1
 * 使用遥控器进行开始校准
 *             第一步:遥控器的两个开关都打到下
 *             第二步:两个摇杆打成\../,保存两秒.\.代表左摇杆向右下打.
 *             第三步:摇杆打成./\. 开始陀螺仪校准
 *                    或者摇杆打成'\/' 开始云台校准
 *                    或者摇杆打成/''\ 开始底盘校准
 *
 *             数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
 *             例如head_cali有八个字节,但它需要12字节在flash,如果它从0x080A0000开始
 *             0x080A0000-0x080A0007: head_cali数据
 *             0x080A0008: 名字name[0]
 *             0x080A0009: 名字name[1]
 *             0x080A000A: 名字name[2]
 *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0x55,意味着head_cali已经校准了
 *             添加新设备
 *             1.添加设备名在calibrate_task.h的cali_id_e, 像
 *             typedef enum
 *             {
 *                 ...
 *                 //add more...
 *                 CALI_XXX,
 *                 CALI_LIST_LENGHT,
 *             } cali_id_e;
 *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
 *
 *             typedef struct
 *             {
 *                 uint16_t xxx;
 *                 uint16_t yyy;
 *                 fp32 zzz;
 *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
 *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
 *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
 *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
 *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
 *
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "calibrate_task.h"
#include "user_c.h"

// include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3 + CALI_LIST_LENGHT * 4)

/**
 * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
 * @param[in]      none
 * @retval         none
 */
static void RC_cmd_to_calibrate(void);

/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_read(void);
 
/**
 * @brief          往flash写入校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_write(void);

/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd); // header device cali function

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd); // gyro device cali function

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); // gimbal device cali function

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static const RC_ctrl_t *calibrate_RC; // remote control point
static head_cali_t head_cali;         // head cali data
static gimbal_cali_t gimbal_cali;     // gimbal cali data
static imu_cali_t accel_cali;         // accel cali data
static imu_cali_t gyro_cali;          // gyro cali data
static imu_cali_t mag_cali;           // mag cali data

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

// cali data address
// 校准数据地址
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
    (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
    (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
    (uint32_t *)&mag_cali};

static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

static uint32_t calibrate_systemTick;

/**
 * @brief          校准任务，由main函数创建
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void calibrate_task(void const *pvParameters)
{
  static uint8_t i = 0;
	
  calibrate_RC = get_remote_ctrl_point_cali();
	
  while (1)
  {
    // 使用遥控器开始校准
    RC_cmd_to_calibrate();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
      if (cali_sensor[i].cali_cmd)
      {
        if (cali_sensor[i].cali_hook != NULL)
        {
          if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
          {
            // done
            cali_sensor[i].name[0] = cali_name[i][0];
            cali_sensor[i].name[1] = cali_name[i][1];
            cali_sensor[i].name[2] = cali_name[i][2];
            // set 0x55
            cali_sensor[i].cali_done = CALIED_FLAG;

            cali_sensor[i].cali_cmd = 0;
            // write
            cali_data_write();
          }
        }
      }
    }
    osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
    calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/**
 * @brief          获取imu控制温度, 单位℃
 * @param[in]      none
 * @retval         imu控制温度
 */
int8_t get_control_temperature(void)
{

  return head_cali.temperature;
}

/**
 * @brief          获取纬度,默认22.0f
 * @param[out]     latitude:fp32指针
 * @retval         none
 */
void get_flash_latitude(float *latitude)
{

  if (latitude == NULL)
  {
    return;
  }
  if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
  {
    *latitude = head_cali.latitude;
  }
  else
  {
    *latitude = 22.0f;
  }
}

/**
 * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
 * @param[in]      none
 * @retval         none
 */
static void RC_cmd_to_calibrate(void)
{
  static const uint8_t BEGIN_FLAG = 1;
  static const uint8_t GIMBAL_FLAG = 2;
  static const uint8_t GYRO_FLAG = 3;
  static const uint8_t CHASSIS_FLAG = 4;

  static uint8_t i;
  static uint32_t rc_cmd_systemTick = 0;
  static uint16_t buzzer_time = 0;
  static uint16_t rc_cmd_time = 0;
  static uint8_t rc_action_flag = 0;

  // if something is calibrating, return
  // 如果已经在校准，就返回
  for (i = 0; i < CALI_LIST_LENGHT; i++)
  {
    if (cali_sensor[i].cali_cmd)
    {
      buzzer_time = 0;
      rc_cmd_time = 0;
      rc_action_flag = 0;

      return;
    }
  }

  if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
  {
    rc_cmd_systemTick = xTaskGetTickCount();
    rc_action_flag = BEGIN_FLAG;
    rc_cmd_time = 0;
  }
  else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
  {
    // 云台校准
    rc_action_flag = 0;
    rc_cmd_time = 0;
    cali_sensor[CALI_GIMBAL].cali_cmd = 1;
    cali_buzzer_off();
  }
  else if (rc_action_flag == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
  {
    // 陀螺仪初始化
    rc_action_flag = 0;
    rc_cmd_time = 0;
    cali_sensor[CALI_GYRO].cali_cmd = 1;
    // 更新控制温度
    head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
      head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    cali_buzzer_off();
  }
  else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
  {
    rc_action_flag = 0;
    rc_cmd_time = 0;

    // 发送CAN重设ID命令到3508
    // CAN_cmd_chassis_reset_ID();
    // CAN_cmd_chassis_reset_ID();
    // CAN_cmd_chassis_reset_ID();
    // cali_buzzer_off();
  }

  if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
  {
    // 两个摇杆打成 \../,保持2s
    rc_cmd_time++;
  }
  else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
  {
    // 两个摇杆打成'\/',保持2s
    rc_cmd_time++;
    rc_action_flag = GIMBAL_FLAG;
  }
  else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
  {
    // 两个摇杆打成./\.,保持2s
    rc_cmd_time++;
    rc_action_flag = GYRO_FLAG;
  }
  else if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
  {
    // 两个摇杆打成/''\,保持2s
    // 电机校准(设置ID) 不需要
    // rc_cmd_time++;
    // rc_action_flag = CHASSIS_FLAG;
  }
  else
  {
    rc_cmd_time = 0;
  }

  calibrate_systemTick = xTaskGetTickCount();

  if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
  {
    // 超过20s,停止
    rc_action_flag = 0;
    return;
  }
  else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
  {
    rc_cali_buzzer_middle_on();
  }
  else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
  {
    rc_cali_buzzer_start_on();
  }

  if (rc_action_flag != 0)
  {
    buzzer_time++;
  }

  if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
  {
    buzzer_time = 0;
  }
  if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
  {
    cali_buzzer_off();
  }
}

/**
 * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
 * @param[in]      none
 * @retval         none
 */
void cali_param_init(void)
{
  uint8_t i = 0;

  for (i = 0; i < CALI_LIST_LENGHT; i++)
  {
    cali_sensor[i].flash_len = cali_sensor_size[i];
    cali_sensor[i].flash_buf = cali_sensor_buf[i];
    cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
  }

  cali_data_read();

  for (i = 0; i < CALI_LIST_LENGHT; i++)
  {
    if (cali_sensor[i].cali_done == CALIED_FLAG)
    {
      if (cali_sensor[i].cali_hook != NULL)
      {
        // 如果已经校准 设置为 init
        cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
      }
    }
  }
}

/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_read(void)
{
  uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
  uint8_t i = 0;
  uint16_t offset = 0;
  for (i = 0; i < CALI_LIST_LENGHT; i++)
  {

    // 读取flash中数据
    cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);

    offset += cali_sensor[i].flash_len * 4;

    // 读取名字和校准位置标志位
    cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);

    cali_sensor[i].name[0] = flash_read_buf[0];
    cali_sensor[i].name[1] = flash_read_buf[1];
    cali_sensor[i].name[2] = flash_read_buf[2];
    cali_sensor[i].cali_done = flash_read_buf[3];

    offset += CALI_SENSOR_HEAD_LEGHT * 4;

    if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
    {
      cali_sensor[i].cali_cmd = 1;
    }
  }
}

/**
 * @brief          往flash写入校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_write(void)
{
  uint8_t i = 0;
  uint16_t offset = 0;

  for (i = 0; i < CALI_LIST_LENGHT; i++)
  {
    // copy the data of device calibration data
    memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
    offset += cali_sensor[i].flash_len * 4;

    // copy the name and "CALI_FLAG" of device
    memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
    offset += CALI_SENSOR_HEAD_LEGHT * 4;
  }

  // erase the page
  cali_flash_erase(FLASH_USER_ADDR, 1);
  // write data
  cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
  head_cali_t *local_cali_t = (head_cali_t *)cali;
  if (cmd == CALI_FUNC_CMD_INIT)
  {
    //        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

    return 1;
  }
  // self id
  local_cali_t->self_id = SELF_ID;
  // imu control temperature
  local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
  // head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
  if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
  {
    local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
  }

  local_cali_t->firmware_version = FIRMWARE_VERSION;
  // shenzhen latitude
  local_cali_t->latitude = 22.0f;

  return 1;
}

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
  imu_cali_t *local_cali_t = (imu_cali_t *)cali;
  if (cmd == CALI_FUNC_CMD_INIT)
  {
    gyro_set_cali(local_cali_t->scale, local_cali_t->offset);

    return 0;
  }
  else if (cmd == CALI_FUNC_CMD_ON)
  {
    static uint16_t count_time = 0;
    gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
    if (count_time > GYRO_CALIBRATE_TIME)
    {
      count_time = 0;
      cali_buzzer_off();
      gyro_cali_enable_control();
      return 1;
    }
    else
    {
      gyro_cali_disable_control(); // 车静止
      imu_start_buzzer();

      return 0;
    }
  }

  return 0;
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{
  gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
  if (cmd == CALI_FUNC_CMD_INIT)
  {
    set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                         local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                         local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);

    return 0;
  }
  else if (cmd == CALI_FUNC_CMD_ON)
  {
    if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                             &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                             &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
    {
      cali_buzzer_off();

      return 1;
    }
    else
    {
      gimbal_start_buzzer();

      return 0;
    }
  }

  return 0;
}
