#ifndef CHASSIS_MOTOR__H__
#define CHASSIS_MOTOR__H__

#include "robot_config.h"

/*底盘电机索引列表*/
typedef enum {
	Chassis_Motor_LF,
	Chassis_Motor_RF,
	Chassis_Motor_LB,
	Chassis_Motor_RB,
	Chassis_Motor_Count,
}chassis_motor_cnt_e;

/*底盘电机信息结构体*/
typedef struct chassis_motor_info_struct {
	uint16_t	angle;/*机械角度*/
	int16_t		speed;/*转速*/
	int16_t		current;/*电流*/

	uint16_t	angle_prev;/*前机械角度*/
	int32_t		angle_sum;/*最终角度*/

	uint8_t		init_flag;/*初始化标志位*/
	uint8_t		offline_cnt;/*离线记数*/
	uint8_t		offline_max_cnt;/*最大离线记数值*/	
} chassis_motor_info_t;

typedef struct chassis_motor_struct{
    chassis_motor_info_t	*info;		 
	drv_can_t				*driver;
    void					(*init)(struct chassis_motor_struct *self);	
    void					(*update)(struct chassis_motor_struct *self, uint8_t *rxBuf);	
    void					(*check)(struct chassis_motor_struct *self);	
    void					(*heart_beat)(struct chassis_motor_struct *self);	
    dvc_work_state_e		work_state;
    dvc_error_e				error;	
    dvc_id_t				id;	
} chassis_motor_t;

extern chassis_motor_info_t Chassis_Motor_Info[Chassis_Motor_Count];
extern chassis_motor_t Chassis_Motor[Chassis_Motor_Count];

#endif

