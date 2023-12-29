#ifndef LAUNCHER__H__
#define LAUNCHER__H__

/* Includes ------------------------------------------------------------------*/
#include "robot_config.h"

#include "launcher_motor.h"
#include "rc_sensor.h"

#include "driver.h"

#include "pid.h"

#include <stdbool.h>


#define DIAL_STEP -31130

#define re_dial_times 0 
#define MAGEZINE_OPEM_TIME 1000

#define SAFE_SPIN_TIME (120)
#define	LIMIT_SPIN_SPEED (1200)
#define DIAL_WARNING_OUTPUT (8000)

#define FIRE_FASTER_ABILITY 0


/*�ж������궨��*/
#define __Have_Fire_Requst Launcher_Info.launcher_flag->Fire_Resqust == true
#define __Launcher_Is_Stop_Mode	Launcher_Info.launcher_mode == Launcher_Mode_Stop
#define __Launcher_Is_Normal_Mode Launcher_Info.launcher_mode == Launcher_Mode_Normal
#define __Launcher_Is_Debug_Mode Launcher_Info.launcher_mode == Launcher_Mode_Debug


/*Ħ����״̬*/
typedef enum{
	Friction_OFF,
	Friction_ON,
} launcher_friction_flag_e;

/*��λ����״̬*/
typedef enum{
	Limit_OFF,
	Limit_ON,
}launcher_limit_flag_e;

/*����״̬*//*��ӯ ���� ��ס*/
typedef enum{
	Bullet_Full,
	Bullet_Empty,
	Bullet_Stuck,
}launche_bullet_chain_flag_e;

/*����״̬*/
typedef enum{
	Fire_Ready,
	Fire_Busy,
}launcher_fire_flag_e;

/*����ģ��״̬��־λ*/
typedef struct{
	bool Fire_Resqust;
	launche_bullet_chain_flag_e	Bullet_Chain_Flag;
	launcher_friction_flag_e 	Friction_Flag;
	launcher_limit_flag_e		Limit_Flag;
	launcher_fire_flag_e 		Fire_Flag;
}launcher_flag_t;

/*������pid�е�target����*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} launcher_motor_pid_t;

/*�����������*/
typedef struct {
	launcher_motor_pid_t		(*motor)[Launcher_Motor_Count];
} launcher_ctrl_t;

/*������������*/
typedef struct {
	launcher_motor_t	*launcher_motor[Launcher_Motor_Count];
	rc_sensor_t		*rc_sensor;
} launcher_dvc_t;


/*������Ϣ����*/
typedef struct {
	launcher_mode_e				launcher_mode;
	launcher_flag_t				*launcher_flag;
}launcher_info_t;


typedef struct launcher{
	launcher_ctrl_t	*controller;		
	launcher_dvc_t	*dev;							
	launcher_info_t	*info;					
	bool			test_open;					
	void			(*init)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}launcher_t;

extern launcher_t Launcher;
extern int Friction_speed;
extern int Fric_3508_speed[6];

#endif

