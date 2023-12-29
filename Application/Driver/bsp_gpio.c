#include "bsp_gpio.h"

#include "imu_sensor.h"

#include "bmi_protocol.h"
#include "ist_protocol.h"


void Bsp_GPIO_EXTI_Handler(uint16_t GPIO_Pin)
{
	/*������ٶȼ��ж�*/
	if(GPIO_Pin == INT1_ACCEL_Pin)
	{	
		BMI088_Accel_INT1_Handler(&BMI088_Info);
	}
	/*�����������ж�*/
	else if(GPIO_Pin == INT3_GYRO_Pin)
	{	
		BMI088_Gyro_INT3_Handler(&BMI088_Info);
	}
	/*����������ж�*/
	else if(GPIO_Pin == DRDY_IST8310_Pin)
	{	
		IST8310_DRDY_Handler(&IST8310_Info);
	}
	else if(GPIO_Pin == KEY_Pin)
	{

	}
}

