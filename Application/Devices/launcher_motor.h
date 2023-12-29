#ifndef LAUNCHER_MOTOR__H__
#define LAUNCHER_MOTOR__H__

#include "robot_config.h"


/*发射电机设备索引*/
typedef enum {
	Launcher_Limit,
	Launcher_Dial,
	Launcher_Frict_L,
	Launcher_Frict_R,
	Launcher_Motor_Count,
}launcher_motor_cnt_e;

typedef struct	launcher_motor_info_struct{
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} launcher_motor_info_t;


typedef struct	launcher_motor_struct{
	launcher_motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct launcher_motor_struct *self);
	void					(*update)(struct launcher_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct launcher_motor_struct *self);	
	void					(*heart_beat)(struct launcher_motor_struct *self);
	dvc_work_state_e		work_state;
	dvc_error_e				error;
	dvc_id_t				id;
} launcher_motor_t;

extern launcher_motor_t	Launcher_Motor[Launcher_Motor_Count];



#endif

