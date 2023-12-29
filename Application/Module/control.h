#ifndef CONTROL__H__
#define CONTROL__H__

#include "robot_config.h"

//#include "gimbal.h"
//#include "launcher.h"

//#include "rc_sensor.h"
//#include "launcher_motor.h"
//#include "gimbal_motor.h"

#include "can_protocol.h"


typedef struct control{
	void 			(*reset)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}control_t;


extern short CAN1_0X1ff_BUF[4];
extern short CAN1_0X200_BUF[4];
extern short CAN1_0X2ff_BUF[4];

extern short CAN2_0X1ff_BUF[4];
extern short CAN2_0X200_BUF[4];
extern short CAN2_0X2ff_BUF[4];

extern control_t control;

#endif

