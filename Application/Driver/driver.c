#include "driver.h"

#include "bsp_usart.h"
#include "bsp_can.h"

#include "robot_config.h"

void Driver_Init(void)
{
#ifdef PRINTF_DATA_ENABLE
	USART1_Init();
#endif
	USART3_Init();
	CAN1_Init();
	CAN2_Init();	
}








