#include "chassis_task.h"

#include "chassis.h"

#include "cmsis_os.h"


int chassis_trance = 0;


void chassis_task(void const * argument)
{
	Chassis.init();
	
	while(1)
	{
		chassis_trance++;

		Chassis.ctrl();

		osDelay(1);
	}
}


