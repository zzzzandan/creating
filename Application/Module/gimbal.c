#include "gimbal.h"
#include "control.h"

#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "imu_sensor.h"

/*��̨ģ����غ���*/
void Gimbal_Init(void);
void Gimbal_Control(void);
void Gimbal_Reset(void);
void Gimbal_Self_Protect(void);
void Gimbal_PID_Control(void);
bool Gimbal_IMU_If_Back_Mid(void);

/*��̨ģ��������ģ����Ϣ*/
void Gimbal_Get_Robot_Info(void);
void Gimbal_Get_Judge_Info(void);
void Gimbal_Get_Rc_Info(void);
void Gimbal_Get_If_On_Rotate_Mode(void);
void Gimbal_Get_Self_Attitude(void);

/*��̨PID���*/
/*��̨���PID�ṹ�������ʼ��*/
static void Gimbal_PID_Params_Init(gimbal_motor_pid_t *pid, uint8_t motor_cnt);
/*��̨PID���㺯��*/
/*�ٶȻ����*/
static void Gimbal_Speed_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx);
/*�ǶȻ����*/
static void Gimbal_Angle_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx);
/*��̨�����ͣ*//*ֹͣPID���*/
static void Gimbal_Stop(gimbal_motor_pid_t *pid);
/*��̨PID���ռ��������*/
static void Gimbal_PID_Output(gimbal_motor_pid_t *pid);



static void Gimbal_Update_Controller(void);

static void Gimbal_Rc_Control(void);
static void Gimbal_Normal_Control(void);



/*��������ָ��*/
drv_can_t				*gimb_drv[Gimbal_Motor_Count];
gimbal_motor_t			*gimb_motor[Gimbal_Motor_Count];
gimbal_motor_info_t		*gimb_motor_info[Gimbal_Motor_Count];

/*��̨ģ����̬��������*/
gimbal_imu_t Gimbal_Imu = {
	.yaw = 0,
	.pitch = 0,
	.roll = 0,
	.rate_yaw = 0,
	.rate_pitch = 0,
	.rate_roll = 0,
};

/*��̨ģ��PID�����ṹ��*/
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

/*��̨ģ�������*/
gimbal_ctrl_t	Gimbal_Controller = {
	.motor = &Gimbal_Motor_Pid,
};

/*��̨ģ�鴫����*/
gimbal_dvc_t	Gimbal_Devices = {
	.gimbal_motor[Gimbal_Pitch_Motor] = &Gimbal_Motor[Gimbal_Pitch_Motor],
	.gimbal_motor[Gimbal_Yaw_Motor] = &Gimbal_Motor[Gimbal_Yaw_Motor],
	.imu_sensor = &Imu_Sensor,
	.rc_sensor = &Rc_Sensor,
	.gimbal_imu_sensor = &Gimbal_Imu,
};

/*��̨ģ����Ϣ*/
gimbal_info_t 	Gimbal_Info = {
	.remote_mode = RC,
	.gimbal_motor_mode = Absolute_Mode,
	.gimbal_mode = Gimbal_Mode_Normal,
	.logic_front = Logic_Front,
	.if_fire_ready = 0,
};

/*��̨ģ��*/
gimbal_t Gimbal = {
	.controller = &Gimbal_Controller,
	.dev = &Gimbal_Devices,
	.info = &Gimbal_Info,
	.reset = Gimbal_Reset,
	.init = Gimbal_Init,
	.ctrl = Gimbal_Control,
	.self_protect = Gimbal_Self_Protect,
};



/*��̨���PID��ά�ṹ�������ʼ��*/
static void Gimbal_PID_Params_Init(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].angle);
		PID_Init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.fdb;
	}	
}
/*��̨�ǶȻ�PID���㺯��*/
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
/*��̨�ٶȻ�PID���㺯��*/
static void Gimbal_Speed_PID_Calc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_e MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	
	PID_Control_Single(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}
/*��̨PID����*/
static void Gimbal_PID_Control(void)
{	
	Gimbal_Angle_PID_Calc(Gimbal_Motor_Pid, Gimbal_Yaw_Motor);
	Gimbal_Angle_PID_Calc(Gimbal_Motor_Pid, Gimbal_Pitch_Motor);
	
	Gimbal_PID_Output(Gimbal_Motor_Pid);
}
/*��̨PID���������*/
static void Gimbal_PID_Output(gimbal_motor_pid_t *pid)
{
	/*�ж�Yaw�����Ƿ�����*/
	if(gimb_motor[Gimbal_Yaw_Motor]->work_state == Dvc_Online) 
	{
		/*����CAN�·�����*/
		CAN1_0X1ff_BUF[0] = (int16_t)(pid[Gimbal_Yaw_Motor].out);
	} 
	else 
	{
		CAN1_0X1ff_BUF[0] = 0;
	}

	/*�ж�Pitch�����Ƿ�����*/
	if(gimb_motor[Gimbal_Pitch_Motor]->work_state == Dvc_Online) 
	{		
		/*����CAN�·�����*/
		CAN1_0X1ff_BUF[1] = (int16_t)(pid[Gimbal_Pitch_Motor].out);
	} 
	else 
	{
		CAN1_0X1ff_BUF[1] = 0;
	}
}

/*��̨ģ�鸴λ*/
void Gimbal_Reset(void)
{

}


/*��̨ģ���ʼ��*/
void Gimbal_Init(void)
{
	/*��ʼ������ָ��*/
	for(uint8_t i = 0; i < Gimbal_Motor_Count; i++){
		/*��̨����*/
		gimb_drv[i] = Gimbal_Devices.gimbal_motor[i]->driver;
		/*��̨�����Ϣ*/
		gimb_motor_info[i] = Gimbal_Devices.gimbal_motor[i]->info;
		/*��̨���*/
		gimb_motor[i] = Gimbal_Devices.gimbal_motor[i];
	}
	Gimbal_Motor_Pid[Gimbal_Pitch_Motor].angle.target = Gimbal_Motor[Gimbal_Pitch_Motor].info->angle_sum;	
	Gimbal_Motor_Pid[Gimbal_Yaw_Motor].angle.target = Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum;
}


/*��̨ģ��������ģ����Ϣ*/
void Gimbal_Get_Info(void)
{
	Gimbal_Get_Robot_Info();
	Gimbal_Get_Judge_Info();
	Gimbal_Get_Rc_Info();
	Gimbal_Get_If_On_Rotate_Mode();
	Gimbal_Get_Self_Attitude();
	Gimbal_Update_Controller();
}


/*----------��̨ģ��ִ�к���----------*/
/*��̨ģ����ƺ���*/
void Gimbal_Control(void)
{
	/*----��Ϣ����----*/
	Gimbal_Get_Info();
	/*----�����޸�----*/ 
	Gimbal_Normal_Control();
	/*----�������----*/
	Gimbal_PID_Control();	
}
/*��̨ģ�����ұ���*/
void Gimbal_Self_Protect(void)
{
	Gimbal_Stop(Gimbal_Motor_Pid);
	Gimbal_PID_Params_Init(Gimbal_Motor_Pid, Gimbal_Motor_Count);
	Gimbal_Get_Info();
}

/*��̨�����ͣ*/
static void Gimbal_Stop(gimbal_motor_pid_t *pid)
{
	/*PID����������*/
	pid[Gimbal_Pitch_Motor].out = 0;
	pid[Gimbal_Yaw_Motor].out = 0;
	
	/*CAN������������*/
	CAN1_0X1ff_BUF[0] = 0;	
	CAN1_0X1ff_BUF[1] = 0;
}

/*----------��̨ģ�鹦��ģ��----------*/
/*----------��̨��ȡ��Ϣ----------*/
/*��̨ģ���ȡϵͳ��Ϣ*/
void Gimbal_Get_Robot_Info(void)
{
	Gimbal_Info.gimbal_mode = Robot.robot_mode->gimbal_mode;
}
/*��̨ģ���ò���ϵͳ��Ϣ*/
void Gimbal_Get_Judge_Info(void)
{

}
float yaw_add,pitch_add = 0;
/*��̨ģ����ң������Ϣ*/
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
/*��̨ģ���ж��Ƿ���С����ģʽ*/
static void Gimbal_Get_If_On_Rotate_Mode(void)
{
	
}
/*��̨ģ��������Ƕ�*/
static void Gimbal_Get_Self_Attitude(void)
{
	
}

/*��̨���������ݸ���*/
static void Gimbal_Update_Controller(void)
{
	/*�ٶȷ���ֵ����*/
	for(uint8_t i=0; i<Gimbal_Motor_Count; i++){
		Gimbal_Motor_Pid[i].speed.fdb = Gimbal_Motor[i].info->speed;
		Gimbal_Motor_Pid[i].angle.fdb = Gimbal_Motor[i].info->angle_sum;
	}
}


/*----------��̨����ģ��----------*/
/*��̨ģ��ң�������ƺ���*/
static void Gimbal_Rc_Control(void)
{
	Gimbal_Motor_Pid[Gimbal_Yaw_Motor].angle.target += (int)yaw_add;
	Gimbal_Motor_Pid[Gimbal_Pitch_Motor].angle.target += (int)pitch_add;		
}
/*��̨ģ����̿��ƺ���*/
static void Gimbal_Key_Control(void)
{


}
/*��̨ģ����ƺ���*/
static void Gimbal_Normal_Control(void)
{
	/*ң��������*/
	Gimbal_Rc_Control();

	/*��������*/
	if(Is_Key_Control)
		Gimbal_Key_Control();
}



