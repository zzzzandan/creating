#include "gimbal_task.h"

#include "gimbal.h"

#include "cmsis_os.h"


void gimbal_task(void const * argument)
{
	Gimbal.init();

	while (1)
	{
		Gimbal.ctrl();
	
		osDelay(1);
	}

}

