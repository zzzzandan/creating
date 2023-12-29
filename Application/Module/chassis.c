#include "chassis.h"
#include "control.h"

#include "chassis_motor.h"
#include "rc_sensor.h"

/*底盘模块相关函数*/
void Chassis_Init(void);
void Chassis_Control(void);
void Chassis_Reset(void);
void Chassis_Self_Protect(void);
void Chassis_PID_Control(void);

/*底盘模块获得其他模块信息*/
void Chassis_Get_Robot_Info(void);
void Chassis_Get_Judge_Info(void);
void Chassis_Get_Rc_Info(void);

/*----------底盘PID相关----------*/
/*底盘电机PID结构体参数初始化*/
static void Chassis_PID_Params_Init(chassis_motor_pid_t *pid, uint8_t motor_cnt);
/*速度环输出*/
static void Chassis_Speed_PID_Calc(chassis_motor_pid_t *pid, chassis_motor_cnt_e MOTORx);
/*底盘电机急停*//*停止PID输出*/
static void Chassis_Stop(chassis_motor_pid_t *pid);
/*底盘PID最终计算结果输出*/
static void Chassis_PID_Output(chassis_motor_pid_t *pid);


/*----------本地驱动指针----------*/
static drv_can_t				*chas_drv[Chassis_Motor_Count];
static chassis_motor_t			*chas_motor[Chassis_Motor_Count];
static chassis_motor_info_t		*chas_motor_info[Chassis_Motor_Count];
static chassis_mode_e			mode;


/*底盘模块PID参数结构体*/
chassis_motor_pid_t Chassis_Motor_Pid[Chassis_Motor_Count] = {
	[Chassis_Motor_LF] = {
		.speed.kp = 10.0f,
		.speed.ki = 0.0f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 6000,
	},
	[Chassis_Motor_RF] = {
		.speed.kp = 10.0f, 
		.speed.ki = 0.0f,
		.speed.kd = 0,
		.speed.integral_max = 16000,
		.speed.out_max = 6000,
	},
	[Chassis_Motor_LB] = {
		.speed.kp = 10.0f, 
		.speed.ki = 0.0f,
		.speed.kd = 0,
		.speed.integral_max = 16000,
		.speed.out_max = 6000,
	},
	[Chassis_Motor_RB] = {
		.speed.kp = 10.0f, 
		.speed.ki = 0.0f,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 6000,
	},
};

/*底盘模块控制体*/
chassis_ctrl_t Chassis_Controller = {
	.motor = &Chassis_Motor_Pid,
};

/*底盘模块传感器*/
chassis_dvc_t	Chassis_Devices = {
	.chassis_motor[Chassis_Motor_LF] = &Chassis_Motor[Chassis_Motor_LF],
	.chassis_motor[Chassis_Motor_RF] = &Chassis_Motor[Chassis_Motor_RF],
	.chassis_motor[Chassis_Motor_LB] = &Chassis_Motor[Chassis_Motor_LB],
	.chassis_motor[Chassis_Motor_RB] = &Chassis_Motor[Chassis_Motor_RB],
	.rc_sensor = &Rc_Sensor,
};

/*底盘模块信息*/
chassis_info_t 	Chassis_Info = {
	.remote_mode = RC,
	.chassis_mode = Chassis_Mode_Paralyze,
	.logic_front = Logic_Front,
};

/*底盘模块*/
chassis_t Chassis = {
	.controller = &Chassis_Controller,
	.dev = &Chassis_Devices,
	.info = &Chassis_Info,
	.reset = Chassis_Reset,
	.init = Chassis_Init,
	.ctrl = Chassis_Control,
	.self_protect = Chassis_Self_Protect,
};

/*----------底盘PID相关函数----------*/
/*底盘电机PID多维结构体参数初始化*/
static void Chassis_PID_Params_Init(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].speed);
		pid[i].out = 0;		
	}	
}
/*底盘速度环PID计算函数*/
static void Chassis_Speed_PID_Calc(chassis_motor_pid_t *pid, chassis_motor_cnt_e MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	
	PID_Control_Single(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}
/*底盘PID计算*/
void Chassis_PID_Control(void)
{	
	for(chassis_motor_cnt_e i = Chassis_Motor_LF; i < Chassis_Motor_Count; i++)
		Chassis_Speed_PID_Calc(Chassis_Motor_Pid,i);

	/*云台模块PID计算结果下发*/
	Chassis_PID_Output(Chassis_Motor_Pid);
}
/*底盘电机停止PID输出*/
static void Chassis_Stop(chassis_motor_pid_t *pid)
{
	/*PID计算结果清零*/
	for(uint8_t i = 0; i < Chassis_Motor_Count; i++)
		pid[i].out = 0;
	
	/*CAN发送数据清零*/
	CAN2_0X200_BUF[0] = 0;
	CAN2_0X200_BUF[1] = 0;
	CAN2_0X200_BUF[2] = 0;
	CAN2_0X200_BUF[3] = 0;
}
/*底盘PID计算结果输出*/
static void Chassis_PID_Output(chassis_motor_pid_t *pid)
{
	/*更新CAN下发数据*/
	if(__Chassis_Is_Paralyze_Mode){
		Chassis_Stop(Chassis_Motor_Pid);
	}
	else{
		CAN2_0X200_BUF[0] = (int16_t)(pid[Chassis_Motor_LF].out);
		CAN2_0X200_BUF[1] = (int16_t)(pid[Chassis_Motor_RF].out);
		CAN2_0X200_BUF[2] = (int16_t)(pid[Chassis_Motor_LB].out);
		CAN2_0X200_BUF[3] = (int16_t)(pid[Chassis_Motor_RB].out);
	}
}


/*----------地盘模块获取信息相关函数----------*/
/*底盘模块获取机器人信息*/
void Chassis_Get_Robot_Info(void)
{
	Chassis_Info.chassis_mode = Robot.robot_mode->chassis_mode;
}
/*底盘模块获得裁判系统信息*/
void Chassis_Get_Judge_Info(void)
{

}

float Max_Speed = 4000.0f;
float temp_x,temp_y,temp_z = 0.0;
/*底盘模块获得遥控器信息*//*通过当前遥控器数据得出百分比*/
void Chassis_Get_Rc_Info(void)
{
	rc_sensor_info_t *info = Chassis_Devices.rc_sensor->info;

	if(__Chassis_Is_Normal_Mode){
		temp_x = info->ch0 * RcChannelToPercentage *  Max_Speed;
		temp_y = info->ch3 * RcChannelToPercentage * -Max_Speed;
		temp_z = info->thumbwheel * RcChannelToPercentage * Max_Speed;
	}
	
	else if(__Chassis_Is_Paralyze_Mode){
		temp_x = 0;
		temp_y = 0;
		temp_z = 0;
	}

	else if(__Chassis_Is_Rotating_Mode){
		Chassis_Info.if_rotate_mode = true;
		temp_z = Max_Speed;
	}
}
/*底盘控制器数据更新*/
void Chassis_Update_Controller(void)
{
	/*速度反馈值更新*/
	for(uint8_t i=0; i<Chassis_Motor_Count; i++){
		Chassis_Motor_Pid[i].speed.fdb = Chassis_Motor[i].info->speed;
	}
}
/*底盘模块获得其他模块信息*/
void Chassis_Get_Info(void)
{
	Chassis_Get_Robot_Info();
	Chassis_Get_Judge_Info();
	Chassis_Get_Rc_Info();
	Chassis_Update_Controller();
}


/*----------底盘模块控制信息相关函数----------*/
/*底盘模块遥控器控制函数*/
void Chassis_Rc_Control(void)
{	
	float speed_set[4] ={ 0.0 };

	/*计算各电机PID目标值*/
	speed_set[0] =  temp_x - temp_y - temp_z;
	speed_set[1] =  temp_x + temp_y - temp_z;
	speed_set[2] = -temp_x - temp_y - temp_z;
	speed_set[3] = -temp_x + temp_y - temp_z;

	/*设定值更新*/
	for(uint8_t i=0; i<Chassis_Motor_Count;i++){
		Chassis_Motor_Pid[i].speed.target = speed_set[i];
	}
}
/*底盘模块键盘控制函数*/
void Chassis_Key_Control(void)
{

	
}
/*底盘正常控制函数*/
void Chassis_Normal_Control(void)
{
	/*遥控器控制*/
	Chassis_Rc_Control();

	/*按键控制*/
	if(Is_Key_Control)
		Chassis_Key_Control();
}


/*----------底盘模块主要控制函数----------*/
/*底盘模块初始化*/
void Chassis_Init(void)
{
	/*初始化本地指针*/
	for(uint8_t i = 0; i < Chassis_Motor_Count; i++){
		/*底盘驱动*/
		chas_drv[i] = Chassis_Devices.chassis_motor[i]->driver;
		/*底盘电机信息*/
		chas_motor_info[i] = Chassis_Devices.chassis_motor[i]->info;
		/*底盘电机*/
		chas_motor[i] = Chassis_Devices.chassis_motor[i];
	}
}

/*底盘模块控制函数*/
void Chassis_Control(void)
{
	/*----信息读入----*/
	Chassis_Get_Info();
	/*----期望修改----*/ 
	Chassis_Normal_Control();
	/*----最终输出----*/
	Chassis_PID_Control();	
}

/*底盘模块复位*/
void Chassis_Reset(void)
{

}

/*底盘模块自我保护*/
void Chassis_Self_Protect(void)
{
	Chassis_Stop(Chassis_Motor_Pid);
	Chassis_PID_Params_Init(Chassis_Motor_Pid, Chassis_Motor_Count);
	Chassis_Get_Info();
}


