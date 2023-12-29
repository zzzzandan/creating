#ifndef CHASSIS__H__
#define CHASSIS__H__

#include "robot_config.h"

#include "chassis_motor.h"
#include "rc_sensor.h"

#include "pid.h"

#include <stdbool.h>

/*遥控器数据转化成百分比系数*/
#define RcChannelToPercentage 0.00151515151515152f

/*判断条件宏定义*/
#define __Chassis_Is_Normal_Mode Chassis_Info.chassis_mode == Chassis_Mode_Normal
#define __Chassis_Is_Paralyze_Mode Chassis_Info.chassis_mode == Chassis_Mode_Paralyze
#define __Chassis_Is_Rotating_Mode Chassis_Info.chassis_mode == Chassis_Mode_Rotating


/*底盘电机PID结构体*//*速度环 角度环 输出*/
typedef struct {
	pid_ctrl_t	speed;
	float		out;
} chassis_motor_pid_t;

/*底盘控制结构体*//*电机的PID结构体*/
typedef struct {
	chassis_motor_pid_t		(*motor)[Chassis_Motor_Count];
} chassis_ctrl_t;

/*底盘设备列表*/
typedef struct {
	chassis_motor_t	*chassis_motor[Chassis_Motor_Count];
	rc_sensor_t		*rc_sensor;
} chassis_dvc_t;

/*底盘信息结构体*/
typedef struct chassis_info{
	remote_mode_e		remote_mode;
	chassis_mode_e		chassis_mode;
	logic_front_e		logic_front;
	bool				if_rotate_mode;
}chassis_info_t;

/*底盘对象结构体*/
typedef struct chassis{
	chassis_ctrl_t	*controller;		
	chassis_dvc_t	*dev;							
	chassis_info_t	*info;					
	void			(*init)(void);
	void 			(*reset)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
}chassis_t;

extern chassis_t Chassis;

#endif

