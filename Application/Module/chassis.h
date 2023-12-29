#ifndef CHASSIS__H__
#define CHASSIS__H__

#include "robot_config.h"

#include "chassis_motor.h"
#include "rc_sensor.h"

#include "pid.h"

#include <stdbool.h>

/*ң��������ת���ɰٷֱ�ϵ��*/
#define RcChannelToPercentage 0.00151515151515152f

/*�ж������궨��*/
#define __Chassis_Is_Normal_Mode Chassis_Info.chassis_mode == Chassis_Mode_Normal
#define __Chassis_Is_Paralyze_Mode Chassis_Info.chassis_mode == Chassis_Mode_Paralyze
#define __Chassis_Is_Rotating_Mode Chassis_Info.chassis_mode == Chassis_Mode_Rotating


/*���̵��PID�ṹ��*//*�ٶȻ� �ǶȻ� ���*/
typedef struct {
	pid_ctrl_t	speed;
	float		out;
} chassis_motor_pid_t;

/*���̿��ƽṹ��*//*�����PID�ṹ��*/
typedef struct {
	chassis_motor_pid_t		(*motor)[Chassis_Motor_Count];
} chassis_ctrl_t;

/*�����豸�б�*/
typedef struct {
	chassis_motor_t	*chassis_motor[Chassis_Motor_Count];
	rc_sensor_t		*rc_sensor;
} chassis_dvc_t;

/*������Ϣ�ṹ��*/
typedef struct chassis_info{
	remote_mode_e		remote_mode;
	chassis_mode_e		chassis_mode;
	logic_front_e		logic_front;
	bool				if_rotate_mode;
}chassis_info_t;

/*���̶���ṹ��*/
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

