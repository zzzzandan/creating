#ifndef GIMBAL__H__
#define GIMBAL__H__

#include "robot_config.h"

#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "imu_sensor.h"

#include "pid.h"
#include "stdbool.h"

/*遥控器数据转化成百分比系数*/
#define RcChannelToPercentage 0.00151515151515152f


/*判断条件宏定义*/
#define __Gimbal_Is_Normal_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Normal
#define __Gimbal_Is_Paralyze_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Paralyze
#define __Gimbal_Is_Stable_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Stable
#define __Gimbal_Is_Vision_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Vision



/*云台电机PID结构体*//*速度环 角度环 输出*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} gimbal_motor_pid_t;

/*云台控制结构体*//*电机的PID结构体*/
typedef struct {
	gimbal_motor_pid_t		(*motor)[Gimbal_Motor_Count];
} gimbal_ctrl_t;

/*云台姿态解算数据结构体*/
typedef struct {
	float yaw;
	float pitch;
	float roll;
	short rate_yaw;
	short rate_pitch;
	short rate_roll;
} gimbal_imu_t;

/*云台设备列表*//*云台电机 陀螺仪 遥控器 IMU*/
typedef struct {
	gimbal_motor_t	*gimbal_motor[Gimbal_Motor_Count];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
	gimbal_imu_t    *gimbal_imu_sensor;
} gimbal_dvc_t;


/*云台信息结构体*/
typedef struct {
	remote_mode_e		remote_mode;
	gimbal_motor_mode_e 		gimbal_motor_mode;
	gimbal_mode_e		gimbal_mode;
	logic_front_e		logic_front; //设置头朝向的位置
	bool				if_rotate_mode;
	bool 				if_fire_ready;
}gimbal_info_t;

/*云台对象结构体*/
typedef struct {
	gimbal_ctrl_t	*controller;		
	gimbal_dvc_t	*dev;							
	gimbal_info_t	*info;					
	bool			test_open;					
	void			(*init)(void);
	void 			(*reset)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}gimbal_t;

extern gimbal_t Gimbal;


#endif

