#ifndef USER_FRIC_H
#define USER_FRIC_H

#include "user_c.h"

#define FRIC_MAX 1320

#define FRIC_UP 1315
#define FRIC_DOWN 1300
#define FRIC_OFF 1000

//定义最小转速，在摩擦轮转动时，用于检测拨弹能否正常，并且威力不大
#define FRIC_MIN	1160
//在摩擦轮工作的时候，该值用于调节单次摩擦轮转速的值
#define FRIC_ONCE	0.1f


#define FRIC_SPEED_MAX 	2.7f
#define FRIC_SPEED_MIN 	2.5f

//单个摩擦电机最大速度
#define MAX_FRIC_SPEED 3.0f


extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

#endif /*USER_FRIC_H*/
