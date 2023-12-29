#include "launcher.h"
#include "control.h"

#include "launcher_motor.h"

#include "cmsis_os.h"

#include <stdlib.h>

/*----------����ģ�麯��----------*/
void Launcher_Init(void);
void Launcher_Control(void);
void Launcher_Test(void);
void Launcher_Reset(void);
void Launcher_Self_Protect(void);

/*----------����ģ���ȡ����ģ����Ϣ����----------*/
static void Launcher_Get_Robot_Info(void);
static void Launcher_Get_Rc_Info(void);
static void Launcher_Update_Controller(void);

/*----------PID��غ���----------*/
/*��ʼ��PID�ڲ�����*/
static void Launcher_Pid_Params_Init(launcher_motor_pid_t *pid, uint8_t motor_cnt);
/*ֹͣPID���*/
static void Launcher_Stop(launcher_motor_pid_t *pid);
/*pid�������*/
static void Launcher_Pid_Output(launcher_motor_pid_t *pid);
/*----------����ID����PID���----------*/
/*�ǶȻ�PID����*/
static void Launcher_Angle_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx);
/*�ٶȻ�PID����*/
static void Launcher_Speed_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx);

/*----------���书�ܺ���----------*/
static void Launcher_Reload_To_Full(void);
static void Launcher_Shoot(void);
static void Launcher_ON_OFF_Fric(void);


/*����ָ��*/
static drv_can_t				*luch_drv[Launcher_Motor_Count];
static launcher_motor_t			*luch_motor[Launcher_Motor_Count];
static launcher_motor_info_t	*luch_motor_info[Launcher_Motor_Count];
static launcher_mode_e 			mode;

/*״̬��־λ*/
static uint8_t fire_press_flag = 0;
static uint8_t friction_press_flag = 0;


static int Friction_speed = 0;
static uint32_t Friction_target = 0;


/*Ħ����M3508ת��*/
static int Fric_3508_speed[6] = {1000, 6000, 3250, 4300, 5580, 4620};//5000; 


/*������PID�ṹ��*/
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


/*------------��ʼ����غ���------------*/
/*��̨���PID������ʼ��*//*��ά�ṹ���ʼ��*/
static void Launcher_Pid_Params_Init(launcher_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].angle);
		PID_Init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.fdb;
	}	
}

/*����ģ���ʼ��*//*��ʼ��ָ�����*/
void Launcher_Init(void)
{
	for(uint8_t i = 0; i < Launcher_Motor_Count; i++){
		/*����ģ������*/
		luch_drv[i] = Launcher_Device.launcher_motor[i]->driver;
		/*����ģ������Ϣ*/
		luch_motor_info[i] = Launcher_Device.launcher_motor[i]->info;
		/*����ģ����*/
		luch_motor[i] = Launcher_Device.launcher_motor[i];
	}

	/*���䵯������*/
	Launcher_Reload_To_Full();
}


/*----------PID������غ���----------*/
/*����ģ���ٶȻ�PID����*/
static void Launcher_Speed_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx)
{
	/*�������ֵ*/
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	/*PID�������Ƽ���*//*�ٶȻ�*/
	PID_Control_Single(&pid[MOTORx].speed);
	/*����ٶȻ�������*/
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/*����ģ��ǶȻ�PID����*/
static void Launcher_Angle_Pid_Calc(launcher_motor_pid_t *pid, launcher_motor_cnt_e MOTORx)
{
	/*����Ƕ����ֵ*/
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.fdb;
	
	/*�ų�����*/
	if(MOTORx == Launcher_Dial)
	{
		if(abs((int)pid[MOTORx].angle.err)<50)
		{
			pid[MOTORx].angle.err = 0;
		}
	}
	
	/*�ǶȻ�PID����*/
	PID_Control_Single(&pid[MOTORx].angle);
	/*�ǶȻ�����ֵ�趨Ϊ�ٶȻ�Ŀ��ֵ*/
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;

	/*�����ٶ����ֵ*/
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.fdb;
	/*�ٶȻ�PID����*/
	PID_Control_Single(&pid[MOTORx].speed);
	/*����ٶȻ�������*/
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/*����ģ��PID����*/
static void Launcher_PID_Control(void)
{	
	/*����PID����*/
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Limit);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Dial);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Frict_L);
	Launcher_Speed_Pid_Calc(Launcher_Motor_PID, Launcher_Frict_R);

	/*PID�������·�CAN���ͻ���*/
	Launcher_Pid_Output(Launcher_Motor_PID);
}
/*����ģ����PID�������·�CAN���ͻ���*/
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


/*----------��ȡ����ģ����Ϣ----------*/
/*����ģ���ȡ��������Ϣ*/
static void Launcher_Get_Robot_Info(void)
{
	 Launcher_Info.launcher_mode =	Robot.robot_mode->launcher_mode;
}
/*����ģ���ȡ����ϵͳ��Ϣ*/
static void Launcher_Get_Judge_Info(void)
{

}
/*����ģ���ȡң������Ϣ*/
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
/*����ģ���������Ϣ����*/
static void Launcher_Update_Controller(void)
{
	for(uint8_t i=0;i<Launcher_Motor_Count;i++){
		Launcher_Motor_PID[i].angle.fdb = Launcher_Motor[i].info->angle_sum;
		Launcher_Motor_PID[i].speed.fdb = Launcher_Motor[i].info->speed;
	}
}
/*����ģ���ȡ��Ϣ*/
static void Launcher_Get_Info(void)
{
	/*��ȡϵͳģ����Ϣ*/
	Launcher_Get_Robot_Info();	
	/*���¿�������Ϣ*/
	Launcher_Update_Controller();
	/*����ң������Ϣ*/
	Launcher_Get_Rc_Info();
}


/*----------������غ���----------*/
/*���伱ͣ����*/
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

/*����ģ�鲹�䵯������*/
static void Launcher_Reload_To_Full(void)
{
	/*��⵱ǰ�����Ƿ��ӯ*/

	/*����*/

	/*ֹͣ����*/	
}

/*��λ�ַ���*/
static void Launcher_Limt_Shoot(void)
{
	if(__Have_Fire_Requst){


	}
}

/*�����ֲ��䵯��*/
uint32_t dial_timeout = 0;
static void Launcher_Dial_Reload(void)
{
	if(__Have_Fire_Requst){
		Launcher_Motor_PID[Launcher_Dial].angle.target = 8190;
	}
}


/*����ģ�鷢��*/
static void Launcher_Shoot(void)
{
	if(__Have_Fire_Requst){
		/*�����ֲ��䵯��*/
		Launcher_Dial_Reload();	

		/*��λ���͵�*/
		Launcher_Limt_Shoot();
	}

	/*�����������*/
	Launcher_Info.launcher_flag->Fire_Resqust = false;
}


/*����Ħ����*//*�趨Ħ����Ŀ��ֵ*/
static void Launcher_ON_OFF_Fric(void)
{
	/*ѡ��Ħ����ת��*/
	Friction_speed = Fric_3508_speed[2];

	/*���ݻ�����״̬�л�Ħ����״̬*/
	if(__Launcher_Is_Stop_Mode)
	{
		Launcher.info->launcher_flag->Friction_Flag = Friction_OFF;
		
		/*Ħ���ֵ��PID����Ŀ��ֵ����*/
		Launcher_Motor_PID[Launcher_Limit].speed.target = 0;
		Launcher_Motor_PID[Launcher_Frict_L].speed.target = 0;
		Launcher_Motor_PID[Launcher_Frict_R].speed.target = 0;
	}
	else if(__Launcher_Is_Normal_Mode)
	{
		Launcher.info->launcher_flag->Friction_Flag = Friction_ON;

		/*Ħ���ֵ��PID����Ŀ��ֵ����*/		
		Launcher_Motor_PID[Launcher_Limit].speed.target = 6000;
		Launcher_Motor_PID[Launcher_Frict_L].speed.target = -Friction_speed;
		Launcher_Motor_PID[Launcher_Frict_R].speed.target =  Friction_speed;
	}	
}


/*----------����ģ�����----------*/
/*����ģ��ң�������ƺ���*/
static void Launcher_Rc_Control(void)
{			
	rc_sensor_info_t *info = Launcher_Device.rc_sensor->info;

	/*����ģ��Ϊ����ģʽ*/
	if(__Launcher_Is_Debug_Mode){
		Launcher.info->launcher_flag->Friction_Flag = Friction_ON;

		Launcher_Motor_PID[Launcher_Limit].speed.target = 7000;

		/*��Ӧ���*/
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
	
	/*����ģ��Ϊֹͣģʽ�������ģʽ*/
	else{
		Launcher_ON_OFF_Fric();
		Launcher_Shoot();
		
		Launcher_Motor_PID[Launcher_Dial].speed.target = dial_speed;
	}

}

/*����ģ����̿��ƺ���*/
static void Launcher_Key_Control(void)
{

}

/*����ģ����������ģ��*/
static void Launcher_Normal_Control(void)
{
	/*����ң������Ϣ���Ʒ���ģ��*/
	Launcher_Rc_Control();
	
	/*���ݼ�����Ϣ���Ʒ���ģ��*/
	if(Is_Key_Control)
		Launcher_Key_Control();
}


/*����ģ����ƺ���*/
void Launcher_Control(void)
{
	/*----��Ϣ����----*/
	Launcher_Get_Info();
	/*----�����޸�----*/ 
	Launcher_Normal_Control();
	/*----�������----*/
	Launcher_PID_Control();	
}

/*����ģ����Ժ���*/
void Launcher_Test(void)
{

}

/*����ģ�鸴λ*/
void Launcher_Reset(void)
{
	launcher_flag_t *flag = Launcher.info->launcher_flag;

	/*���ÿ�������*/
	flag->Fire_Resqust = false;

	/*���÷���ģ���־λ*/
	flag->Fire_Flag = Fire_Busy;
	flag->Bullet_Chain_Flag = Bullet_Empty;
	flag->Friction_Flag = Friction_OFF;
	flag->Limit_Flag = Limit_OFF;
}

/*����ģ�����ұ���*/
void Launcher_Self_Protect(void)
{
	/*Ħ����״̬��־λ*/
	Launcher.info->launcher_flag->Friction_Flag = Friction_OFF;

	/*ֹͣPID���*/
	Launcher_Stop(Launcher_Motor_PID);
	/*����ģ��PID�ṹ������*/
	Launcher_Pid_Params_Init(Launcher_Motor_PID, Launcher_Motor_Count);
	/*����ģ���ȡ��Ϣ*/
	Launcher_Get_Info();
}

