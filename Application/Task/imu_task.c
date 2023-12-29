#include "imu_task.h"
#include "imu_sensor.h"
#include "bmi_protocol.h"

#include "cmsis_os.h"

int imu_count;
void imu_task(void const * argument)
{

    while(1)
		{
			imu_count++;
		
				
	    	osDelay(1);
		}
}










