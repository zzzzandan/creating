#include "bsp_pwm.h"

#include "tim.h"

extern TIM_HandleTypeDef htim10;

void BSP_BMI088_Tempture_Control(int set)
{
	__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,set);
}



