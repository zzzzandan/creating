#ifndef GIMBAL__H__
#define GIMBAL__H__

#include "robot_config.h"

#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "imu_sensor.h"

#include "pid.h"
#include "stdbool.h"

/*ң��������ת���ɰٷֱ�ϵ��*/
#define RcChannelToPercentage 0.00151515151515152f


/*�ж������궨��*/
#define __Gimbal_Is_Normal_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Normal
#define __Gimbal_Is_Paralyze_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Paralyze
#define __Gimbal_Is_Stable_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Stable
#define __Gimbal_Is_Vision_Mode Gimbal_Info.gimbal_mode == Gimbal_Mode_Vision



/*��̨���PID�ṹ��*//*�ٶȻ� �ǶȻ� ���*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} gimbal_motor_pid_t;

/*��̨���ƽṹ��*//*�����PID�ṹ��*/
typedef struct {
	gimbal_motor_pid_t		(*motor)[Gimbal_Motor_Count];
} gimbal_ctrl_t;

/*��̨��̬�������ݽṹ��*/
typedef struct {
	float yaw;
	float pitch;
	float roll;
	short rate_yaw;
	short rate_pitch;
	short rate_roll;
} gimbal_imu_t;

/*��̨�豸�б�*//*��̨��� ������ ң���� IMU*/
typedef struct {
	gimbal_motor_t	*gimbal_motor[Gimbal_Motor_Count];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
	gimbal_imu_t    *gimbal_imu_sensor;
} gimbal_dvc_t;


/*��̨��Ϣ�ṹ��*/
typedef struct {
	remote_mode_e		remote_mode;
	gimbal_motor_mode_e 		gimbal_motor_mode;
	gimbal_mode_e		gimbal_mode;
	logic_front_e		logic_front; //����ͷ�����λ��
	bool				if_rotate_mode;
	bool 				if_fire_ready;
}gimbal_info_t;

/*��̨����ṹ��*/
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

