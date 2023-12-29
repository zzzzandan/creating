#include "system_task.h"

#include "launcher.h"
#include "rc_sensor.h"

#include "robot_config.h"

#include "cmsis_os.h"
#include "stdbool.h"


static void Robot_Get_Info(void);
static void Robot_Mode_Switch(void);


/*�����˸�ģ��ģʽ*/
robot_mode_t Robot_Mode = {
	.gimbal_mode = Gimbal_Mode_Paralyze,
	.launcher_mode = Launcher_Mode_Stop,
	.chassis_mode = Chassis_Mode_Paralyze,
};

/*������ģʽ*/
robot_t Robot = {
	.robot_mode = &Robot_Mode,
	.remote_mode = RC,
	.state = Robot_State_Normal,
	.buff = Robot_Buff_Normal,
};


/*�����˻����Ϣ*/
static void Robot_Get_Info(void)
{
	/*ң����ʧ��*/
	if(__Rcsensor_Is_Offline){
		Robot.state = Robot_State_RC_Lost;
		RC_Reset_Data(&Rc_Sensor);
		Robot.remote_mode = RC;
	} 
	
	/*ң��������*/
	else if(Rc_Sensor.work_state == Dvc_Online){
		/*ң��������*/
		if(Rc_Sensor.error == Dvc_None_Error){
			/*ʧ���ָ�*/
			if(Robot.state == Robot_State_RC_Lost){		
				/*�л���ǰ״̬*/
				Robot.remote_mode = RC;
				Robot.state = Robot_State_Normal;
				/*�л���̨ģ��Ϊ����ģʽ*/
				Robot.robot_mode->gimbal_mode = Gimbal_Mode_Normal;
				/*�л�����ģ��Ϊ����ģʽ*/
				Robot.robot_mode->chassis_mode = Chassis_Mode_Normal;
			}
			
			/*һ������*/
			else if(Robot.state == Robot_State_Normal){	
				/*�л���ģ��ģʽ*/
				Robot_Mode_Switch();
			}				
		}
	}
}

/*����ң�����л���ģ��ģʽ*/
static void Robot_Mode_Switch_RC(void)
{	
	/*���´˴β�ť��ֵ*/
	uint8_t sw1 = RC_SW1_VALUE;
	uint8_t sw2 = RC_SW2_VALUE;

	/*��ťΪ��*//*����ģ��*/
	if(sw1 == RC_SW_UP)
	{		
		/*�Ҳ�ťΪ��*//*��������ģʽ*/
		if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Normal;
		}
		/*�Ҳ�ťΪ��*//*����ֹͣģʽ*/
		else if(sw2 == RC_SW_MID)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Stop;
		}
		/*�Ҳ�ťΪ��*//*�������ģʽ*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Debug;
			Robot.robot_mode->chassis_mode = Chassis_Mode_Paralyze;		
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Paralyze;
		}
	}

	/*��ťΪ��*//*��̨ģ��*/
	else if(sw1 == RC_SW_MID)
	{
		/*�Ҳ�ťΪ��*//*��̨����ģʽ*/
		if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Normal;
		}
		/*�Ҳ�ťΪ��*//*��̨����ģʽ*/
		else if(sw2 == RC_SW_MID)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Paralyze;
		}
		/*�Ҳ�ťΪ��*//*��̨����ģʽ*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Vision;
		}
	}

	/*��ťΪ��*//*����ģ��*/
	else if(sw1 == RC_SW_DOWN)
	{
		/*�Ҳ�ťΪ��*//*����ģʽ*/
		if(sw2 == RC_SW_MID | Robot.robot_mode->launcher_mode == Launcher_Mode_Debug)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Paralyze;
		}		
		/*�Ҳ�ťΪ��*//*����ģʽ*/
		else if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Normal;
		}
		/*�Ҳ�ťΪ��*//*С����ģʽ*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Rotating;
		}
	}
}

/*���ݼ����л���ģ��ģʽ*/
static void Robot_Mode_Switch_KEY(void)
{

}

/*�л���ģ��ģʽ*/
static void Robot_Mode_Switch(void)
{
	/*����ң�����л���ģ��ģʽ*/
	Robot_Mode_Switch_RC();
	
	/*���ݼ����л���ģ��ģʽ*/
	if(Robot.remote_mode == KEY)
		Robot_Mode_Switch_KEY();
}

int robot_trancy = 0;
/*�����˾�������*/
void robot_task(void const * argument)
{
	while(1)
	{
		robot_trancy++;
		/*�����ٽ���*/
		portENTER_CRITICAL();
		
		/*�����˸�����Ϣ*/
		Robot_Get_Info();

		/*�뿪�ٽ���*/
		portEXIT_CRITICAL();

		osDelay(1);
	}
}


