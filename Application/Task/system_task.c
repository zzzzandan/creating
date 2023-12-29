#include "system_task.h"

#include "launcher.h"
#include "rc_sensor.h"

#include "robot_config.h"

#include "cmsis_os.h"
#include "stdbool.h"


static void Robot_Get_Info(void);
static void Robot_Mode_Switch(void);


/*机器人各模块模式*/
robot_mode_t Robot_Mode = {
	.gimbal_mode = Gimbal_Mode_Paralyze,
	.launcher_mode = Launcher_Mode_Stop,
	.chassis_mode = Chassis_Mode_Paralyze,
};

/*机器人模式*/
robot_t Robot = {
	.robot_mode = &Robot_Mode,
	.remote_mode = RC,
	.state = Robot_State_Normal,
	.buff = Robot_Buff_Normal,
};


/*机器人获得信息*/
static void Robot_Get_Info(void)
{
	/*遥控器失联*/
	if(__Rcsensor_Is_Offline){
		Robot.state = Robot_State_RC_Lost;
		RC_Reset_Data(&Rc_Sensor);
		Robot.remote_mode = RC;
	} 
	
	/*遥控器在线*/
	else if(Rc_Sensor.work_state == Dvc_Online){
		/*遥控器正常*/
		if(Rc_Sensor.error == Dvc_None_Error){
			/*失联恢复*/
			if(Robot.state == Robot_State_RC_Lost){		
				/*切换当前状态*/
				Robot.remote_mode = RC;
				Robot.state = Robot_State_Normal;
				/*切换云台模块为正常模式*/
				Robot.robot_mode->gimbal_mode = Gimbal_Mode_Normal;
				/*切换底盘模块为正常模式*/
				Robot.robot_mode->chassis_mode = Chassis_Mode_Normal;
			}
			
			/*一切正常*/
			else if(Robot.state == Robot_State_Normal){	
				/*切换各模块模式*/
				Robot_Mode_Switch();
			}				
		}
	}
}

/*根据遥控器切换各模块模式*/
static void Robot_Mode_Switch_RC(void)
{	
	/*更新此次拨钮的值*/
	uint8_t sw1 = RC_SW1_VALUE;
	uint8_t sw2 = RC_SW2_VALUE;

	/*左拨钮为上*//*发射模块*/
	if(sw1 == RC_SW_UP)
	{		
		/*右拨钮为上*//*发射正常模式*/
		if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Normal;
		}
		/*右拨钮为中*//*发射停止模式*/
		else if(sw2 == RC_SW_MID)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Stop;
		}
		/*右拨钮为上*//*发射调试模式*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->launcher_mode = Launcher_Mode_Debug;
			Robot.robot_mode->chassis_mode = Chassis_Mode_Paralyze;		
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Paralyze;
		}
	}

	/*左拨钮为中*//*云台模块*/
	else if(sw1 == RC_SW_MID)
	{
		/*右拨钮为上*//*云台正常模式*/
		if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Normal;
		}
		/*右拨钮为中*//*云台无力模式*/
		else if(sw2 == RC_SW_MID)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Paralyze;
		}
		/*右拨钮为下*//*云台自瞄模式*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->gimbal_mode = Gimbal_Mode_Vision;
		}
	}

	/*左拨钮为下*//*底盘模块*/
	else if(sw1 == RC_SW_DOWN)
	{
		/*右拨钮为中*//*无力模式*/
		if(sw2 == RC_SW_MID | Robot.robot_mode->launcher_mode == Launcher_Mode_Debug)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Paralyze;
		}		
		/*右拨钮为上*//*正常模式*/
		else if(sw2 == RC_SW_UP)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Normal;
		}
		/*右拨钮为下*//*小陀螺模式*/
		else if(sw2 == RC_SW_DOWN)
		{
			Robot.robot_mode->chassis_mode = Chassis_Mode_Rotating;
		}
	}
}

/*根据键盘切换各模块模式*/
static void Robot_Mode_Switch_KEY(void)
{

}

/*切换各模块模式*/
static void Robot_Mode_Switch(void)
{
	/*根据遥控器切换各模块模式*/
	Robot_Mode_Switch_RC();
	
	/*根据键盘切换各模块模式*/
	if(Robot.remote_mode == KEY)
		Robot_Mode_Switch_KEY();
}

int robot_trancy = 0;
/*机器人决策任务*/
void robot_task(void const * argument)
{
	while(1)
	{
		robot_trancy++;
		/*进入临界区*/
		portENTER_CRITICAL();
		
		/*机器人更新信息*/
		Robot_Get_Info();

		/*离开临界区*/
		portEXIT_CRITICAL();

		osDelay(1);
	}
}


