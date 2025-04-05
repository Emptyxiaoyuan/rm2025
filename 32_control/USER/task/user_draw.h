/**
 * @file user_draw.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 * @date 2023-04-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef USER_DRAW_H
#define USER_DRAW_H

#include "user_c.h"

#define FH_SOF 0xA5
#define FH_COM_ID 0x0301
/*-************************赛前修改TODO:****************************-*/
/**
 * @brief 机器人ID
 */
#define RED_HERO_ID				1
#define	RED_PROJECT_ID		2
#define RED_INFANTRY3_ID	3
#define RED_INFANTRY4_ID	4
#define RED_INFANTRY5_ID	5
#define RED_AERIAL_ROBOTICS_ID	6
#define	RED_SENTRY_ID			7

#define BLUE_HERO_ID				101
#define	BLUE_PROJECT_ID			102
#define BLUE_INFANTRY3_ID		103
#define BLUE_INFANTRY4_ID		104
#define BLUE_INFANTRY5_ID		105
#define BLUE_AERIAL_ROBOTICS_ID		106
#define	BLUE_SENTRY_ID			107



#define NOW_ID 5

/**
 * @brief 红方客户端ID
 */
 
#define	CLIENT_HERO_RED			 0x0101
#define CLIENT_PROJECT_RED	 0x0102
#define CLIENT_INFANTRY3_RED 0x0103
#define CLIENT_INFANTRY4_RED 0x0104
#define CLIENT_INFANTRY5_RED 0x0105
/**
 * @brief 蓝方客户端ID
 */
#define	CLIENT_HERO_BLUE			0x0165
#define CLIENT_PROJECT_BLUE	 	0x0166
#define CLIENT_INFANTRY3_BLUE 0x0167 
#define CLIENT_INFANTRY4_BLUE 0x0168
#define CLIENT_INFANTRY5_BLUE 0x0169

/*-****************************************************-*/
#define COMMUNICATION_ID 0x0301    // 学生机器人间通信ID
#define DRAW_SEVEN_GRAPH_ID 0x0104 // 绘制7个图形ID
#define DRAW_CHARACTER_ID 0x0110   // 绘制字符ID

#define CLIENT_LENGHT 1920 // 屏幕长
#define CLIENT_WIDTH 1080  // 屏幕宽

void draw_UI_task(void const *argument);


#define SEND_MESSAGE(message, length) usart6_tx_dma_enable(message, length)



#define PRIMITIVE_CAT(x, y) x ## y
#define CAT(x, y) PRIMITIVE_CAT(x, y)

#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef __packed struct {                                        \
uint8_t figure_name[3];                                 \
uint32_t operate_tpyel:3;                               \
uint32_t figure_tpye:3;                                 \
uint32_t layer:4;                                       \
uint32_t color:4;                                       \
uint32_t PRIMITIVE_CAT(,p_a) :9;                        \
uint32_t PRIMITIVE_CAT(,p_b):9;                         \
uint32_t width:10;                                      \
uint32_t start_x:11;                                    \
uint32_t start_y:11;                                    \
uint32_t PRIMITIVE_CAT(,p_c):10;                        \
uint32_t PRIMITIVE_CAT(,p_d):11;                        \
uint32_t PRIMITIVE_CAT(,p_e):11;                        \
} ui_interface_ ## name ##_t

DEFINE_MESSAGE(figure, _a, _b, _c, _d, _e);
DEFINE_MESSAGE(line, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(rect, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(round, _a, _b, r, _d, _e);
DEFINE_MESSAGE(ellipse, _a, _b, _c, rx, ry);
DEFINE_MESSAGE(arc, start_angle, end_angle, _c, rx, ry);

typedef __packed struct {
    uint8_t figure_name[3];
    uint32_t operate_tpyel: 3;
    uint32_t figure_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t _b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    int32_t number;
} ui_interface_number_t;

typedef __packed struct {
    uint8_t figure_name[3];
    uint32_t operate_tpyel: 3;
    uint32_t figure_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t str_length: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t _c: 10;
    uint32_t _d: 11;
    uint32_t _e: 11;
    char string[30];
} ui_interface_string_t;

typedef __packed struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id, sub_id;
    uint16_t send_id, recv_id;
} ui_frame_header_t;

#define DEFINE_FIGURE_MESSAGE(num)      \
typedef __packed struct {                        \
ui_frame_header_t header;               \
ui_interface_figure_t data[num];        \
uint16_t crc16;                         \
} ui_ ## num##_frame_t

DEFINE_FIGURE_MESSAGE(1);
DEFINE_FIGURE_MESSAGE(2);
DEFINE_FIGURE_MESSAGE(5);
DEFINE_FIGURE_MESSAGE(7);

typedef __packed struct {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
}ui_string_frame_t;

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);


extern ui_interface_line_t *ui_1_Ungroup_NewLine1;
extern ui_interface_line_t *ui_1_Ungroup_NewLine2;
extern ui_interface_line_t *ui_1_Ungroup_NewLine3;
extern ui_interface_line_t *ui_1_Ungroup_NewLine4;
extern ui_interface_number_t *ui_1_Ungroup_NewNumber;

void _ui_init_1_Ungroup_0(void);
void _ui_update_1_Ungroup_0(void);
void _ui_remove_1_Ungroup_0(void);

#endif // USER_DRAW_H
