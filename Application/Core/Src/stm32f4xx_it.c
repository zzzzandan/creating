/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "gpio.h"

#include "bmi_protocol.h"

#include "bsp_usart.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern TIM_HandleTypeDef htim3;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern UART_HandleTypeDef huart3;

extern bmi088_info_t BMI088_Info;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{	while (1){}}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{   while (1){}}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{	while (1){}}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{	while (1){}}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{   while (1){}}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/*���������ж�*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
}
/*���������ݾ����ж�*/
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
/*���ٶȼ����ݾ����ж�*/
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
/*���������ݾ����ж�*/
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}


/*��Ϊϵͳʱ��*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}


/*����һ����DMAͨ���ж�*//*vofa+*/
//void DMA2_Stream7_IRQHandler(void)
///{
//  HAL_DMA_IRQHandler(&hdma_usart1_tx);
//}


/*This function handles CAN1 RX0 interrupts.*/
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}
/*This function handles CAN2 RX0 interrupts.*/
void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}


/*C��DBUS����*/
void USART3_IRQHandler(void)
{
	Bsp_Uart_IRQHandler(&huart3);
	HAL_UART_IRQHandler(&huart3);
}


/*�ⲿ�жϻص�����*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Bsp_GPIO_EXTI_Handler(GPIO_Pin);
}


/*SPI1��DMA��������ж�*/
void DMA2_Stream2_IRQHandler(void)
{
	/*�������ȡ��Ƭѡ*/
	BMI088_ACCEL_NS_H();
	BMI088_GYRO_NS_H();
	
	BMI088_Data_Ready_Handler(&BMI088_Info);
	
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

/*SPI1��DMA��������ж�*/
void DMA2_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
}


