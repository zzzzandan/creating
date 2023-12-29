#include "gimbal.h"
#include "control.h"

#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "imu_sensor.h"

/*云台模块相关函数*/
void Gimbal_Init(void);
void Gimbal_Control(void);
void Gimbal_Reset(void);
void Gimbal_Self_Protect(void);
void Gimbal_PID_Control(void);
bool Gimbal_IMU_If_Back_Mid(void);

/*云台模块获得其他模块信息*/
void Gimbal_Get_Robot_Info(void);
void Gimbal_Get_Judge_Info(void);
void Gimbal_Get_Rc_Info(void);
void Gimbal_Get_If_On_Rotate_Mode(void);
void Gimbal_Get_Self_Attitude(void);

/*云台PID相关*/
/*云台电机PID结构体参数初始化*/
static void Gimbal_PID_Params_Init(gimbal_motor_pid_t *pid, uint8_t motor_cnt);
/*云台PID计算函数*/
/*速度环输出*/
static void Gimbal_Speed_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx);
/*角度环输出*/
static void Gimbal_Angle_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx);
/*云台电机急停*//*停止PID输出*/
static void Gimbal_Stop(gimbal_motor_pid_t *pid);
/*云台PID最终计算结果输出*/
static void Gimbal_PID_Output(gimbal_motor_pid_t *pid);



static void Gimbal_Update_Controller(void);

static void Gimbal_Rc_Control(void);
static void Gimbal_Normal_Control(void);



/*本地驱动指针*/
drv_can_t				*gimb_drv[Gimbal_Motor_Count];
gimbal_motor_t			*gimb_motor[Gimbal_Motor_Count];
gimbal_motor_info_t		*gimb_motor_info[Gimbal_Motor_Count];

/*云台模块姿态解算数据*/
gimbal_imu_t Gimbal_Imu = {
	.yaw = 0,
	.pitch = 0,
	.roll = 0,
	.rate_yaw = 0,
	.rate_pitch = 0,
	.rate_roll = 0,
};

/*云台模块PID参数结构体*/
gimbal_motor_pid_t 	Gimbal_Motor_Pid[Gimbal_Motor_Count] = {
	[Gimbal_Pitch_Motor] = {
		.speed.kp = 50.0f, //35.0f,
		.speed.ki = 0.0f,
		.speed.kd = 0.0f,
		.speed.integral_max = 0,
		.speed.out_max = 30000,
		.angle.kp = 1.0f, //30.f,
		.angle.ki = 0.0f,
		.angle.kd = 0.0f,
		.angle.integral_max = 2000,
		.angle.out_max = 20000,
	},
	[Gimbal_Yaw_Motor] = {
		.speed.kp = 50.0f, 
		.speed.ki = 0.0f,
		.speed.kd = 70.0f,
		.speed.integral_max = 500,
		.speed.out_max = 28000,
		.angle.kp = 0.37f, 
		.angle.ki = 0.075f,
		.angle.kd = 0.8f,
		.angle.integral_max = 800,
		.angle.out_max = 3800,
	},
};

/*云台模块控制体*/
gimbal_ctrl_t	Gimbal_Controller = {
	.motor = &Gimbal_Motor_Pid,
};

/*云台模块传感器*/
gimbal_dvc_t	Gimbal_Devices = {
	.gimbal_motor[Gimbal_Pitch_Motor] = &Gimbal_Motor[Gimbal_Pitch_Motor],
	.gimbal_motor[Gimbal_Yaw_Motor] = &Gimbal_Motor[Gimbal_Yaw_Motor],
	.imu_sensor = &Imu_Sensor,
	.rc_sensor = &Rc_Sensor,
	.gimbal_imu_sensor = &Gimbal_Imu,
};

/*云台模块信息*/
gimbal_info_t 	Gimbal_Info = {
	.remote_mode = RC,
	.gimbal_motor_mode = Absolute_Mode,
	.gimbal_mode = Gimbal_Mode_Normal,
	.logic_front = Logic_Front,
	.if_fire_ready = 0,
};

/*云台模块*/
gimbal_t Gimbal = {
	.controller = &Gimbal_Controller,
	.dev = &Gimbal_Devices,
	.info = &Gimbal_Info,
	.reset = Gimbal_Reset,
	.init = Gimbal_Init,
	.ctrl = Gimbal_Control,
	.self_protect = Gimbal_Self_Protect,
};



/*云台电机PID多维结构体参数初始化*/
static void Gimbal_PID_Params_Init(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].angle);
		PID_Init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.fdb;
	}	
}
/*云台角度环PID计算函数*/
static void Gimbal_Angle_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.fdb;
	
//	if(MOTORx == Gimbal_Yaw_Motor)
//	{
//		if(pid[MOTORx].angle.err>GIMBAL_YAW_CIRCULAR_STEP/2)
//		{
//			pid[MOTORx].angle.err -= GIMBAL_YAW_CIRCULAR_STEP;
//		}
//		else if(pid[MOTORx].angle.err<-GIMBAL_YAW_CIRCULAR_STEP/2)
//		{
//			pid[MOTORx].angle.err += GIMBAL_YAW_CIRCULAR_STEP;
//		}
//	}

	if(abs((int)pid[MOTORx].angle.err) < 20)
		pid[MOTORx].angle.err = 0;

	pid[MOTORx].angle.true_err = pid[MOTORx].angle.err;
	
	PID_Control_Single(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;

	if(abs((int)pid[MOTORx].speed.err) < 20)
		pid[MOTORx].speed.err = 0;
	
	PID_Control_Single(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}
/*云台速度环PID计算函数*/
static void Gimbal_Speed_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	
	PID_Control_Single(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}
/*云台PID计算*/
static void Gimbal_PID_Control(void)
{	
	Gimbal_Angle_PID_Calc(Gimbal_Motor_Pid, Gimbal_Yaw_Motor);
	Gimbal_Angle_PID_Calc(Gimbal_Motor_Pid, Gimbal_Pitch_Motor);
	
	Gimbal_PID_Output(Gimbal_Motor_Pid);
}
/*云台PID计算结果输出*/
static void Gimbal_PID_Output(gimbal_motor_pid_t *pid)
{
	/*判断Yaw轴电机是否在线*/
	if(gimb_motor[Gimbal_Yaw_Motor]->work_state == Dvc_Online) 
	{
		/*更新CAN下发数据*/
		CAN1_0X1ff_BUF[0] = (int16_t)(pid[Gimbal_Yaw_Motor].out);
	} 
	else 
	{
		CAN1_0X1ff_BUF[0] = 0;
	}

	/*判断Pitch轴电机是否在线*/
	if(gimb_motor[Gimbal_Pitch_Motor]->work_state == Dvc_Online) 
	{		
		/*更新CAN下发数据*/
		CAN1_0X1ff_BUF[1] = (int16_t)(pid[Gimbal_Pitch_Motor].out);
	} 
	else 
	{
		CAN1_0X1ff_BUF[1] = 0;
	}
}

/*云台模块复位*/
void Gimbal_Reset(void)
{

}


/*云台模块初始化*/
void Gimbal_Init(void)
{
	/*初始化本地指针*/
	for(uint8_t i = 0; i < Gimbal_Motor_Count; i++){
		/*云台驱动*/
		gimb_drv[i] = Gimbal_Devices.gimbal_motor[i]->driver;
		/*云台电机信息*/
		gimb_motor_info[i] = Gimbal_Devices.gimbal_motor[i]->info;
		/*云台电机*/
		gimb_motor[i] = Gimbal_Devices.gimbal_motor[i];
	}
	Gimbal_Motor_Pid[Gimbal_Pitch_Motor].angle.target = Gimbal_Motor[Gimbal_Pitch_Motor].info->angle_sum;	
	Gimbal_Motor_Pid[Gimbal_Yaw_Motor].angle.target = Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum;
}


/*云台模块获得其他模块信息*/
void Gimbal_Get_Info(void)
{
	Gimbal_Get_Robot_Info();
	Gimbal_Get_Judge_Info();
	Gimbal_Get_Rc_Info();
	Gimbal_Get_If_On_Rotate_Mode();
	Gimbal_Get_Self_Attitude();
	Gimbal_Update_Controller();
}


/*----------云台模块执行函数----------*/
/*云台模块控制函数*/
void Gimbal_Control(void)
{
	/*----信息读入----*/
	Gimbal_Get_Info();
	/*----期望修改----*/ 
	Gimbal_Normal_Control();
	/*----最终输出----*/
	Gimbal_PID_Control();	
}
/*云台模块自我保护*/
void Gimbal_Self_Protect(void)
{
	Gimbal_Stop(Gimbal_Motor_Pid);
	Gimbal_PID_Params_Init(Gimbal_Motor_Pid, Gimbal_Motor_Count);
	Gimbal_Get_Info();
}

/*云台电机急停*/
static void Gimbal_Stop(gimbal_motor_pid_t *pid)
{
	/*PID计算结果清零*/
	pid[Gimbal_Pitch_Motor].out = 0;
	pid[Gimbal_Yaw_Motor].out = 0;
	
	/*CAN发送数据清零*/
	CAN1_0X1ff_BUF[0] = 0;	
	CAN1_0X1ff_BUF[1] = 0;
}

/*----------云台模块功能模块----------*/
/*----------云台获取信息----------*/
/*云台模块获取系统信息*/
void Gimbal_Get_Robot_Info(void)
{
	Gimbal_Info.gimbal_mode = Robot.robot_mode->gimbal_mode;
}
/*云台模块获得裁判系统信息*/
void Gimbal_Get_Judge_Info(void)
{

}
float yaw_add,pitch_add = 0;
/*云台模块获得遥控器信息*/
void Gimbal_Get_Rc_Info(void)
{
	rc_sensor_info_t *info = Gimbal_Devices.rc_sensor->info;
	if(__Gimbal_Is_Normal_Mode){
		yaw_add = info->ch2 * RcChannelToPercentage * 10;
		pitch_add = info->ch1 * RcChannelToPercentage * 10;
	}
	else if(__Gimbal_Is_Paralyze_Mode){
		yaw_add = 0;
		pitch_add = 0;
	}
	else if(__Gimbal_Is_Stable_Mode){

	}
}
/*云台模块判断是否处于小陀螺模式*/
static void Gimbal_Get_If_On_Rotate_Mode(void)
{
	
}
/*云台模块获得自身角度*/
static void Gimbal_Get_Self_Attitude(void)
{
	
}

/*云台控制器数据更新*/
static void Gimbal_Update_Controller(void)
{
	/*速度反馈值更新*/
	for(uint8_t i=0; i<Gimbal_Motor_Count; i++){
		Gimbal_Motor_Pid[i].speed.fdb = Gimbal_Motor[i].info->speed;
		Gimbal_Motor_Pid[i].angle.fdb = Gimbal_Motor[i].info->angle_sum;
	}
}


/*----------云台控制模块----------*/
/*云台模块遥控器控制函数*/
static void Gimbal_Rc_Control(void)
{
	Gimbal_Motor_Pid[Gimbal_Yaw_Motor].angle.target += (int)yaw_add;
	Gimbal_Motor_Pid[Gimbal_Pitch_Motor].angle.target += (int)pitch_add;		
}
/*云台模块键盘控制函数*/
static void Gimbal_Key_Control(void)
{


}
/*云台模块控制函数*/
static void Gimbal_Normal_Control(void)
{
	/*遥控器控制*/
	Gimbal_Rc_Control();

	/*按键控制*/
	if(Is_Key_Control)
		Gimbal_Key_Control();
}



