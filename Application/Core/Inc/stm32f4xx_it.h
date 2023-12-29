/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);


void TIM3_IRQHandler(void);


void EXTI0_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);

/*����һ����DMAͨ���ж�*//*vofa+*/
//void DMA2_Stream7_IRQHandler(void);

/*SPI1��DMA�ж�*/
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
/*SPI�ж�*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);



#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
