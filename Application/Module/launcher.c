#include "launcher.h"
#include "control.h"

#include "launcher_motor.h"

#include "cmsis_os.h"

#include <stdlib.h>

/*----------发射模块函数----------*/
void Launcher_Init(void);
void Launcher_Control(void);
void Launcher_Test(void);
void Launcher_Reset(void);
void Launcher_Self_Protect(void);

/*----------发射模块获取其他模块信息函数----------*/
static void Launcher_Get_Robot_Info(void);
static void Launcher_Get_Rc_Info(void);
static void Launcher_Update_Controller(void);

/*----------PID相关函数----------*/
/*初始化PID内部数据*/
static void Launcher_Pid_Params_Init(launcher_motor_pid_t *pid, uint8_t motor_cnt);
/*停止PID输出*/
static void Launcher_Stop(launcher_motor_pid_t *pid);
/*pid输出函数*/
static void Launcher_Pid_Output(launcher_motor_pid_t *pid);
/*----------根据ID计算PID输出----------*/
/*角度环PID计算*/
static void Launcher_Angle_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx);
/*速度环PID计算*/
static void Launcher_Speed_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx);

/*----------发射功能函数----------*/
static void Launcher_Reload_To_Full(void);
static void Launcher_Shoot(void);
static void Launcher_ON_OFF_Fric(void);


/*本地指针*/
static drv_can_t				*luch_drv[Launcher_Motor_Count];
static launcher_motor_t			*luch_motor[Launcher_Motor_Count];
static launcher_motor_info_t	*luch_motor_info[Launcher_Motor_Count];
static launcher_mode_e 			mode;

/*状态标志位*/
static uint8_t fire_press_flag = 0;
static uint8_t friction_press_flag = 0;


static int Friction_speed = 0;
static uint32_t Friction_target = 0;


/*摩擦轮M3508转速*/
static int Fric_3508_speed[6] = {1000, 6000, 3250, 4300, 5580, 4620};//5000; 


/*发射电机PID结构体*/
launcher_motor_pid_t Launcher_Motor_PID[Launcher_Motor_Count] = {
	[Launcher_Limit] = {
		.speed.kp = 35.0f, //35.0f,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 60,
		.speed.out_max = 3000,
		.speed.target = 500,
		.angle.kp = 1.0f, //30.f,
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 100,
		.angle.out_max = 5000,
	},
	[Launcher_Dial] = {
		.speed.kp = 20.0f, //22.0f,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 6000.0f,
		.speed.out_max = 8000.0f,
		.angle.kp = 10.0f,//22.0f,
		.angle.ki = 0,
		.angle.kd = 0.0f,
		.angle.integral_max = 0,
		.angle.out_max = 8000.0f,
	},
	[Launcher_Frict_L] = {
		.speed.kp = 20.0f, 
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.out_max = 9000,
		.angle.kp = 0.2f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 500,
		.angle.out_max = 8000,
	},
	[Launcher_Frict_R] = {
		.speed.kp = 20.0f, 
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.out_max = 10000,
		.angle.kp = 0.2f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 500,
		.angle.out_max = 8000,
	},
};

launcher_ctrl_t		Launcher_Ctrl = {
	.motor = &Launcher_Motor_PID,
};

launcher_dvc_t		Launcher_Device = {
	.launcher_motor[Launcher_Limit] = &Launcher_Motor[Launcher_Limit],
	.launcher_motor[Launcher_Dial] = &Launcher_Motor[Launcher_Dial],
	.launcher_motor[Launcher_Frict_L] = &Launcher_Motor[Launcher_Frict_L],
	.launcher_motor[Launcher_Frict_R] = &Launcher_Motor[Launcher_Frict_R],
	.rc_sensor = &Rc_Sensor,
};

launcher_flag_t Launcher_Flag = {
	.Fire_Resqust = false,
	.Bullet_Chain_Flag = Bullet_Empty,
	.Friction_Flag = Friction_OFF,
	.Limit_Flag = Limit_OFF,
	.Fire_Flag = Fire_Busy,
};

launcher_info_t Launcher_Info = {
	.launcher_mode = Launcher_Mode_Stop,
	.launcher_flag = &Launcher_Flag,
};

launcher_t Launcher = {
	.controller = &Launcher_Ctrl,
	.dev = &Launcher_Device,
	.info = &Launcher_Info,
	.init = Launcher_Init,
	.test = Launcher_Test,
	.ctrl = Launcher_Control,
	.self_protect = Launcher_Self_Protect,
};


/*------------初始化相关函数------------*/
/*云台电机PID参数初始化*//*多维结构体初始化*/
static void Launcher_Pid_Params_Init(launcher_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].angle);
		PID_Init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.fdb;
	}	
}

/*发射模块初始化*//*初始化指针对象*/
void Launcher_Init(void)
{
	for(uint8_t i = 0; i < Launcher_Motor_Count; i++){
		/*发射模块驱动*/
		luch_drv[i] = Launcher_Device.launcher_motor[i]->driver;
		/*发射模块电机信息*/
		luch_motor_info[i] = Launcher_Device.launcher_motor[i]->info;
		/*发射模块电机*/
		luch_motor[i] = Launcher_Device.launcher_motor[i];
	}

	/*补充弹链至满*/
	Launcher_Reload_To_Full();
}


/*----------PID计算相关函数----------*/
/*发射模块速度环PID计算*/
static void Launcher_Speed_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx)
{
	/*计算误差值*/
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	/*PID单环控制计算*//*速度环*/
	PID_Control_Single(&pid[MOTORx].speed);
	/*获得速度环计算结果*/
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/*发射模块角度环PID计算*/
static void Launcher_Angle_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx)
{
	/*计算角度误差值*/
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.fdb;
	
	/*排除干扰*/
	if(MOTORx == Launcher_Dial)
	{
		if(abs((int)pid[MOTORx].angle.err)<50)
		{
			pid[MOTORx].angle.err = 0;
		}
	}
	
	/*角度环PID计算*/
	PID_Control_Single(&pid[MOTORx].angle);
	/*角度环计算值设定为速度环目标值*/
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;

	/*计算速度误差值*/
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	/*速度环PID计算*/
	PID_Control_Single(&pid[MOTORx].speed);
	/*获得速度环计算结果*/
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/*发射模块PID计算*/
static void Launcher_PID_Control(void)
{	
	/*进行PID计算*/
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Limit);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Dial);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Frict_L);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Frict_R);

	/*PID计算结果下放CAN发送缓存*/
	Launcher_Pid_Output(Launcher_Motor_PID);
}
/*发射模块电机PID计算结果下发CAN发送缓存*/
static void Launcher_Pid_Output(launcher_motor_pid_t *pid)
{
	if(Launcher_Motor[Launcher_Limit].work_state == Dvc_Online)
	{
		CAN1_0X200_BUF[0] = (int16_t)(pid[Launcher_Limit].out);
	}
	else 
	{
		CAN1_0X200_BUF[0] = 0;
	}
	
	if(Launcher_Motor[Launcher_Dial].work_state == Dvc_Online)
	{
		CAN2_0X1ff_BUF[0] = (int16_t)(pid[Launcher_Dial].out);
	}
	else 
	{
		CAN2_0X1ff_BUF[0] = 0;
	}
	
	if(Launcher_Motor[Launcher_Frict_L].work_state == Dvc_Online)
	{
		CAN1_0X200_BUF[1] = (int16_t)(pid[Launcher_Frict_L].out);
	}
	else 
	{
		CAN1_0X200_BUF[1] = 0;
	}
	
	if(Launcher_Motor[Launcher_Frict_R].work_state == Dvc_Online)
	{
		CAN1_0X200_BUF[2] = (int16_t)(pid[Launcher_Frict_R].out);
	}
	else 
	{
		CAN1_0X200_BUF[2] = 0;
	}
}


/*----------获取其他模块信息----------*/
/*发射模块获取机器人信息*/
static void Launcher_Get_Robot_Info(void)
{
	 Launcher_Info.launcher_mode =	Robot.robot_mode->launcher_mode;
}
/*发射模块获取裁判系统信息*/
static void Launcher_Get_Judge_Info(void)
{

}
/*发射模块获取遥控器信息*/
float left_add,right_add = 0;
float left_bug_set,right_bug_set = 0;
uint32_t fire_timeout = 0;
float dial_speed = 0;
static void Launcher_Get_Rc_Info(void)
{
	rc_sensor_info_t *info = Launcher_Device.rc_sensor->info;
	launcher_flag_t *flag = Launcher.info->launcher_flag;

	if(__Launcher_Is_Debug_Mode){
		left_add = (float)(info->ch3) * 0.00151515151515152;
		right_add = (float)(info->ch1) * 0.00151515151515152;

		dial_speed = (float)(info->thumbwheel) * 0.151515151515152 * 8;

		left_bug_set -= left_add;
		right_bug_set += right_add;

		left_add = 0;
		right_add = 0;
	}
	else if(__Launcher_Is_Normal_Mode || info->ch1 >= 650)
	{
		dial_speed = (float)(info->thumbwheel) * 0.151515151515152 * 8;
//		if(osKernelSysTick() > fire_timeout)
//		fire_timeout = osKernelSysTick() + 180;
//			flag->Fire_Resqust = true;
	}
}
/*发射模块控制器信息更新*/
static void Launcher_Update_Controller(void)
{
	for(uint8_t i=0;i<Launcher_Motor_Count;i++){
		Launcher_Motor_PID[i].angle.fdb = Launcher_Motor[i].info->angle_sum;
		Launcher_Motor_PID[i].speed.fdb = Launcher_Motor[i].info->speed;
	}
}
/*发射模块获取信息*/
static void Launcher_Get_Info(void)
{
	/*获取系统模块信息*/
	Launcher_Get_Robot_Info();	
	/*更新控制器信息*/
	Launcher_Update_Controller();
	/*更新遥控器信息*/
	Launcher_Get_Rc_Info();
}


/*----------发射相关函数----------*/
/*发射急停函数*/
static void Launcher_Stop(launcher_motor_pid_t *pid)
{
	pid[Launcher_Limit].out = 0;
	pid[Launcher_Dial].out = 0;
	pid[Launcher_Frict_L].out = 0;
	pid[Launcher_Frict_R].out = 0;
	
	CAN1_0X200_BUF[0] = 0;
	CAN1_0X200_BUF[1] = 0;
	CAN1_0X200_BUF[2] = 0;
	CAN1_0X200_BUF[3] = 0;
	CAN2_0X1ff_BUF[0] = 0;
}

/*发射模块补充弹链至满*/
static void Launcher_Reload_To_Full(void)
{
	/*检测当前弹链是否充盈*/

	/*拨弹*/

	/*停止拨弹*/	
}

/*限位轮发射*/
static void Launcher_Limt_Shoot(void)
{
	if(__Have_Fire_Requst){


	}
}

/*拨弹轮补充弹链*/
uint32_t dial_timeout = 0;
static void Launcher_Dial_Reload(void)
{
	if(__Have_Fire_Requst){
		Launcher_Motor_PID[Launcher_Dial].angle.target = 8190;
	}
}


/*发射模块发射*/
static void Launcher_Shoot(void)
{
	if(__Have_Fire_Requst){
		/*拨弹轮补充弹链*/
		Launcher_Dial_Reload();	

		/*限位轮送弹*/
		Launcher_Limt_Shoot();
	}

	/*清除发射请求*/
	Launcher_Info.launcher_flag->Fire_Resqust = false;
}


/*开关摩擦轮*//*设定摩擦轮目标值*/
static void Launcher_ON_OFF_Fric(void)
{
	/*选择摩擦轮转速*/
	Friction_speed = Fric_3508_speed[2];

	/*根据机器人状态切换摩擦轮状态*/
	if(__Launcher_Is_Stop_Mode)
	{
		Launcher.info->launcher_flag->Friction_Flag = Friction_OFF;
		
		/*摩擦轮电机PID计算目标值处理*/
		Launcher_Motor_PID[Launcher_Limit].speed.target = 0;
		Launcher_Motor_PID[Launcher_Frict_L].speed.target = 0;
		Launcher_Motor_PID[Launcher_Frict_R].speed.target = 0;
	}
	else if(__Launcher_Is_Normal_Mode)
	{
		Launcher.info->launcher_flag->Friction_Flag = Friction_ON;

		/*摩擦轮电机PID计算目标值处理*/		
		Launcher_Motor_PID[Launcher_Limit].speed.target = 6000;
		Launcher_Motor_PID[Launcher_Frict_L].speed.target = -Friction_speed;
		Launcher_Motor_PID[Launcher_Frict_R].speed.target =  Friction_speed;
	}	
}


/*----------发射模块控制----------*/
/*发射模块遥控器控制函数*/
static void Launcher_Rc_Control(void)
{			
	rc_sensor_info_t *info = Launcher_Device.rc_sensor->info;

	/*发射模块为调试模式*/
	if(__Launcher_Is_Debug_Mode){
		Launcher.info->launcher_flag->Friction_Flag = Friction_ON;

		Launcher_Motor_PID[Launcher_Limit].speed.target = 7000;

		/*对应左侧*/
		if(info->ch2 <= -650)
		{	
			left_bug_set = -right_bug_set;
		}
		else if(info->ch2 >= 650)
		{			
			right_bug_set = -left_bug_set;
		}
		else if(info->ch0 >= 650)
		{
			right_bug_set = 0;
		}
		else if(info->ch0 <= -650)
		{
			left_bug_set = 0;
		}

		Launcher_Motor_PID[Launcher_Frict_L].speed.target = left_bug_set;
		Launcher_Motor_PID[Launcher_Frict_R].speed.target = right_bug_set;

		Launcher_Motor_PID[Launcher_Dial].speed.target = dial_speed;
	}
	
	/*发射模块为停止模式或发射就绪模式*/
	else{
		Launcher_ON_OFF_Fric();
		Launcher_Shoot();
		
		Launcher_Motor_PID[Launcher_Dial].speed.target = dial_speed;
	}

}

/*发射模块键盘控制函数*/
static void Launcher_Key_Control(void)
{

}

/*发射模块正常控制模块*/
static void Launcher_Normal_Control(void)
{
	/*根据遥控器信息控制发射模块*/
	Launcher_Rc_Control();
	
	/*根据键盘信息控制发射模块*/
	if(Is_Key_Control)
		Launcher_Key_Control();
}


/*发射模块控制函数*/
void Launcher_Control(void)
{
	/*----信息读入----*/
	Launcher_Get_Info();
	/*----期望修改----*/ 
	Launcher_Normal_Control();
	/*----最终输出----*/
	Launcher_PID_Control();	
}

/*发射模块测试函数*/
void Launcher_Test(void)
{

}

/*发射模块复位*/
void Launcher_Reset(void)
{
	launcher_flag_t *flag = Launcher.info->launcher_flag;

	/*重置开火请求*/
	flag->Fire_Resqust = false;

	/*重置发射模块标志位*/
	flag->Fire_Flag = Fire_Busy;
	flag->Bullet_Chain_Flag = Bullet_Empty;
	flag->Friction_Flag = Friction_OFF;
	flag->Limit_Flag = Limit_OFF;
}

/*发射模块自我保护*/
void Launcher_Self_Protect(void)
{
	/*摩擦轮状态标志位*/
	Launcher.info->launcher_flag->Friction_Flag = Friction_OFF;

	/*停止PID输出*/
	Launcher_Stop(Launcher_Motor_PID);
	/*发射模块PID结构体清零*/
	Launcher_Pid_Params_Init(Launcher_Motor_PID, Launcher_Motor_Count);
	/*发射模块获取信息*/
	Launcher_Get_Info();
}

