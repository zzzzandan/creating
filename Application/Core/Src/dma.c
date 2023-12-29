#include "dma.h"


void MX_DMA_Init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();


	/*SPI1��DMA��������ж�*/
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/*SPI1��DMA��������ж�*/
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

	/* DMA2_Stream7_IRQn interrupt configuration *//*����һ������λ������*/
	//  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
	//  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}



