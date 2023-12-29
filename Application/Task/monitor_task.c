#include "monitor_task.h"

#include "device.h"

#include "bmi_protocol.h"

#include "robot_config.h"

#include "cmsis_os.h"


pid_ctrl_t BMI088_Tempture_PID = {
	.target = 40.0f,
	.kp = 1600.0f,
	.ki = 0.2f,
	.kd = 0.0f,
	.integral_max = 4400.0f,
	.out_max = 4999.0f,
};



int monitor_task_count = 0;
void monitor_task(void const * argument)
{
	osDelay(200);
	
	while(1)
	{
		monitor_task_count++;

		BMI088_Tempture_Control(&BMI088_Info,&BMI088_Tempture_PID);
		
		Devices_Heart_Beat();
		osDelay(1);
	}
}




