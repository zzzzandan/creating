#ifndef BSP_USART__H__
#define BSP_USART__H__

#include "stm32f4xx_hal.h"

#include "robot_config.h"


#define USART1_TX_BUF_LEN (10u)

void Bsp_Uart_IRQHandler(UART_HandleTypeDef *huart);


void USART1_Init(void);
void USART3_Init(void);

#ifdef PRINTF_DATA_ENABLE
	extern uint8_t usart1_dma_txbuf[USART1_TX_BUF_LEN];
#endif


#endif

