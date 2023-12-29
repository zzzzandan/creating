#include "launcher_task.h"
#include "launcher.h"
#include "launcher_motor.h"


#include "cmsis_os.h"

int launcher_trance = 0;
void launcher_task(void const * argument)
{
	Launcher.init();
	
	for(;;)
	{
		launcher_trance++;
		
		Launcher.ctrl();
		osDelay(1); 	 
	}
}













