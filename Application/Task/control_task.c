#include "control_task.h"

#include "device.h"

#include "cmsis_os.h"

int control_trance = 0;

void control_task(void const * argument)
{
	control.self_protect();
	
	while(1)
	{
		control_trance++;
		
		control.output();
		osDelay(1);
	}
}

