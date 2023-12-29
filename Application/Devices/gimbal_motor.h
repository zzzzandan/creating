#ifndef GIMBAL_MOTOR__H__
#define GIMBAL_MOTOR__H__

#include "robot_config.h"

/*云台转一周对应的电机角度*/
#define GIMBAL_YAW_CIRCULAR_STEP 8191

typedef enum {
	Gimbal_Yaw_Motor,
	Gimbal_Pitch_Motor,
	Gimbal_Motor_Count,
}gimbal_motor_cnt_e;

/*云台电机模式*//*绝对角度模式 相对角度模式*/
typedef enum {
	Absolute_Mode,
	Relative_Mode,
}gimbal_motor_mode_e;

/*云台电机信息结构体*/
typedef struct gimbal_motor_info_struct {
	uint16_t	angle;/*机械角度*/
	int16_t		speed;/*转速*/
	int16_t		current;/*电流*/
	uint16_t	angle_prev;/*前机械角度*/
	int32_t		angle_sum;/*最终角度*/
	uint8_t		init_flag;/*初始化标志位*/
	uint8_t		offline_cnt;/*离线记数*/
	uint8_t		offline_max_cnt;/*最大离线记数值*/	
} gimbal_motor_info_t;


typedef struct gimbal_motor_struct{
    gimbal_motor_info_t		*info;		 
		drv_can_t				*driver;
    void					(*init)(struct gimbal_motor_struct *self);	
    void					(*update)(struct gimbal_motor_struct *self, uint8_t *rxBuf);	
    void					(*check)(struct gimbal_motor_struct *self);	
    void					(*heart_beat)(struct gimbal_motor_struct *self);	
    dvc_work_state_e		work_state;
    dvc_error_e				error;	
    dvc_id_t				id;	
} gimbal_motor_t;

extern gimbal_motor_info_t Gimbal_Motor_Info[Gimbal_Motor_Count];
extern gimbal_motor_t Gimbal_Motor[Gimbal_Motor_Count];
extern gimbal_motor_info_t gimbal_yaw, gimbal_pitch;




#endif

