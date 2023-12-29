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


/*判断条件宏定义*/
#define __Have_Fire_Requst Launcher_Info.launcher_flag->Fire_Resqust == true
#define __Launcher_Is_Stop_Mode	Launcher_Info.launcher_mode == Launcher_Mode_Stop
#define __Launcher_Is_Normal_Mode Launcher_Info.launcher_mode == Launcher_Mode_Normal
#define __Launcher_Is_Debug_Mode Launcher_Info.launcher_mode == Launcher_Mode_Debug


/*摩擦轮状态*/
typedef enum{
	Friction_OFF,
	Friction_ON,
} launcher_friction_flag_e;

/*限位胶轮状态*/
typedef enum{
	Limit_OFF,
	Limit_ON,
}launcher_limit_flag_e;

/*弹链状态*//*充盈 不足 卡住*/
typedef enum{
	Bullet_Full,
	Bullet_Empty,
	Bullet_Stuck,
}launche_bullet_chain_flag_e;

/*开火状态*/
typedef enum{
	Fire_Ready,
	Fire_Busy,
}launcher_fire_flag_e;

/*发射模块状态标志位*/
typedef struct{
	bool Fire_Resqust;
	launche_bullet_chain_flag_e	Bullet_Chain_Flag;
	launcher_friction_flag_e 	Friction_Flag;
	launcher_limit_flag_e		Limit_Flag;
	launcher_fire_flag_e 		Fire_Flag;
}launcher_flag_t;

/*发射电机pid中的target设置*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} launcher_motor_pid_t;

/*发射控制样例*/
typedef struct {
	launcher_motor_pid_t		(*motor)[Launcher_Motor_Count];
} launcher_ctrl_t;

/*发射驱动样例*/
typedef struct {
	launcher_motor_t	*launcher_motor[Launcher_Motor_Count];
	rc_sensor_t		*rc_sensor;
} launcher_dvc_t;


/*发射信息样例*/
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

