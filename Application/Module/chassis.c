#include "chassis.h"
#include "control.h"

#include "chassis_motor.h"
#include "rc_sensor.h"

/*����ģ����غ���*/
void Chassis_Init(void);
void Chassis_Control(void);
void Chassis_Reset(void);
void Chassis_Self_Protect(void);
void Chassis_PID_Control(void);

/*����ģ��������ģ����Ϣ*/
void Chassis_Get_Robot_Info(void);
void Chassis_Get_Judge_Info(void);
void Chassis_Get_Rc_Info(void);

/*----------����PID���----------*/
/*���̵��PID�ṹ�������ʼ��*/
static void Chassis_PID_Params_Init(chassis_motor_pid_t *pid, uint8_t motor_cnt);
/*�ٶȻ����*/
static void Chassis_Speed_PID_Calc(chassis_motor_pid_t *pid, chassis_motor_cnt_e MOTORx);
/*���̵����ͣ*//*ֹͣPID���*/
static void Chassis_Stop(chassis_motor_pid_t *pid);
/*����PID���ռ��������*/
static void Chassis_PID_Output(chassis_motor_pid_t *pid);


/*----------��������ָ��----------*/
static drv_can_t				*chas_drv[Chassis_Motor_Count];
static chassis_motor_t			*chas_motor[Chassis_Motor_Count];
static chassis_motor_info_t		*chas_motor_info[Chassis_Motor_Count];
static chassis_mode_e			mode;


/*����ģ��PID�����ṹ��*/
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

/*����ģ�������*/
chassis_ctrl_t Chassis_Controller = {
	.motor = &Chassis_Motor_Pid,
};

/*����ģ�鴫����*/
chassis_dvc_t	Chassis_Devices = {
	.chassis_motor[Chassis_Motor_LF] = &Chassis_Motor[Chassis_Motor_LF],
	.chassis_motor[Chassis_Motor_RF] = &Chassis_Motor[Chassis_Motor_RF],
	.chassis_motor[Chassis_Motor_LB] = &Chassis_Motor[Chassis_Motor_LB],
	.chassis_motor[Chassis_Motor_RB] = &Chassis_Motor[Chassis_Motor_RB],
	.rc_sensor = &Rc_Sensor,
};

/*����ģ����Ϣ*/
chassis_info_t 	Chassis_Info = {
	.remote_mode = RC,
	.chassis_mode = Chassis_Mode_Paralyze,
	.logic_front = Logic_Front,
};

/*����ģ��*/
chassis_t Chassis = {
	.controller = &Chassis_Controller,
	.dev = &Chassis_Devices,
	.info = &Chassis_Info,
	.reset = Chassis_Reset,
	.init = Chassis_Init,
	.ctrl = Chassis_Control,
	.self_protect = Chassis_Self_Protect,
};

/*----------����PID��غ���----------*/
/*���̵��PID��ά�ṹ�������ʼ��*/
static void Chassis_PID_Params_Init(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].speed);
		pid[i].out = 0;		
	}	
}
/*�����ٶȻ�PID���㺯��*/
static void Chassis_Speed_PID_Calc(chassis_motor_pid_t *pid, chassis_motor_cnt_e MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	
	PID_Control_Single(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}
/*����PID����*/
void Chassis_PID_Control(void)
{	
	for(chassis_motor_cnt_e i = Chassis_Motor_LF; i < Chassis_Motor_Count; i++)
		Chassis_Speed_PID_Calc(Chassis_Motor_Pid,i);

	/*��̨ģ��PID�������·�*/
	Chassis_PID_Output(Chassis_Motor_Pid);
}
/*���̵��ֹͣPID���*/
static void Chassis_Stop(chassis_motor_pid_t *pid)
{
	/*PID����������*/
	for(uint8_t i = 0; i < Chassis_Motor_Count; i++)
		pid[i].out = 0;
	
	/*CAN������������*/
	CAN2_0X200_BUF[0] = 0;
	CAN2_0X200_BUF[1] = 0;
	CAN2_0X200_BUF[2] = 0;
	CAN2_0X200_BUF[3] = 0;
}
/*����PID���������*/
static void Chassis_PID_Output(chassis_motor_pid_t *pid)
{
	/*����CAN�·�����*/
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


/*----------����ģ���ȡ��Ϣ��غ���----------*/
/*����ģ���ȡ��������Ϣ*/
void Chassis_Get_Robot_Info(void)
{
	Chassis_Info.chassis_mode = Robot.robot_mode->chassis_mode;
}
/*����ģ���ò���ϵͳ��Ϣ*/
void Chassis_Get_Judge_Info(void)
{

}

float Max_Speed = 4000.0f;
float temp_x,temp_y,temp_z = 0.0;
/*����ģ����ң������Ϣ*//*ͨ����ǰң�������ݵó��ٷֱ�*/
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
/*���̿��������ݸ���*/
void Chassis_Update_Controller(void)
{
	/*�ٶȷ���ֵ����*/
	for(uint8_t i=0; i<Chassis_Motor_Count; i++){
		Chassis_Motor_Pid[i].speed.fdb = Chassis_Motor[i].info->speed;
	}
}
/*����ģ��������ģ����Ϣ*/
void Chassis_Get_Info(void)
{
	Chassis_Get_Robot_Info();
	Chassis_Get_Judge_Info();
	Chassis_Get_Rc_Info();
	Chassis_Update_Controller();
}


/*----------����ģ�������Ϣ��غ���----------*/
/*����ģ��ң�������ƺ���*/
void Chassis_Rc_Control(void)
{	
	float speed_set[4] ={ 0.0 };

	/*��������PIDĿ��ֵ*/
	speed_set[0] =  temp_x - temp_y - temp_z;
	speed_set[1] =  temp_x + temp_y - temp_z;
	speed_set[2] = -temp_x - temp_y - temp_z;
	speed_set[3] = -temp_x + temp_y - temp_z;

	/*�趨ֵ����*/
	for(uint8_t i=0; i<Chassis_Motor_Count;i++){
		Chassis_Motor_Pid[i].speed.target = speed_set[i];
	}
}
/*����ģ����̿��ƺ���*/
void Chassis_Key_Control(void)
{

	
}
/*�����������ƺ���*/
void Chassis_Normal_Control(void)
{
	/*ң��������*/
	Chassis_Rc_Control();

	/*��������*/
	if(Is_Key_Control)
		Chassis_Key_Control();
}


/*----------����ģ����Ҫ���ƺ���----------*/
/*����ģ���ʼ��*/
void Chassis_Init(void)
{
	/*��ʼ������ָ��*/
	for(uint8_t i = 0; i < Chassis_Motor_Count; i++){
		/*��������*/
		chas_drv[i] = Chassis_Devices.chassis_motor[i]->driver;
		/*���̵����Ϣ*/
		chas_motor_info[i] = Chassis_Devices.chassis_motor[i]->info;
		/*���̵��*/
		chas_motor[i] = Chassis_Devices.chassis_motor[i];
	}
}

/*����ģ����ƺ���*/
void Chassis_Control(void)
{
	/*----��Ϣ����----*/
	Chassis_Get_Info();
	/*----�����޸�----*/ 
	Chassis_Normal_Control();
	/*----�������----*/
	Chassis_PID_Control();	
}

/*����ģ�鸴λ*/
void Chassis_Reset(void)
{

}

/*����ģ�����ұ���*/
void Chassis_Self_Protect(void)
{
	Chassis_Stop(Chassis_Motor_Pid);
	Chassis_PID_Params_Init(Chassis_Motor_Pid, Chassis_Motor_Count);
	Chassis_Get_Info();
}


