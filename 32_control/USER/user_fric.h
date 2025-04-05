#ifndef USER_FRIC_H
#define USER_FRIC_H

#include "user_c.h"

#define FRIC_MAX 1320

#define FRIC_UP 1315
#define FRIC_DOWN 1300
#define FRIC_OFF 1000

//������Сת�٣���Ħ����ת��ʱ�����ڼ�Ⲧ���ܷ�������������������
#define FRIC_MIN	1160
//��Ħ���ֹ�����ʱ�򣬸�ֵ���ڵ��ڵ���Ħ����ת�ٵ�ֵ
#define FRIC_ONCE	0.1f


#define FRIC_SPEED_MAX 	2.7f
#define FRIC_SPEED_MIN 	2.5f

//����Ħ���������ٶ�
#define MAX_FRIC_SPEED 3.0f


extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

#endif /*USER_FRIC_H*/
