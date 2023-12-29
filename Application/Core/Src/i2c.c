/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

//I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

DMA_HandleTypeDef hdma_i2c3_rx;


/*OLED屏幕*/
//void MX_I2C2_Init(void)
//{
//  hi2c2.Instance = I2C2;
//  hi2c2.Init.ClockSpeed = 400000;
//  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c2.Init.OwnAddress1 = 0;
//  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 // hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c2.Init.OwnAddress2 = 0;
//  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}


/*I2C3初始化*//*磁力计使用I2C3*/
void MX_I2C3_Init(void)
{
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//  if(i2cHandle->Instance==I2C2)
	//  {
	//    __HAL_RCC_GPIOF_CLK_ENABLE();
	//    /**I2C2 GPIO Configuration
	//    PF0     ------> I2C2_SDA
	//    PF1     ------> I2C2_SCL
	//    */
	//    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	//    GPIO_InitStruct.Pull = GPIO_NOPULL;
	//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	//    /* I2C2 clock enable */
	//    __HAL_RCC_I2C2_CLK_ENABLE();
	//  }
	if(i2cHandle->Instance==I2C3)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**I2C3 GPIO Configuration    
		PC9     ------> I2C3_SDA
		PA8     ------> I2C3_SCL 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* I2C3 clock enable */
		__HAL_RCC_I2C3_CLK_ENABLE();
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  if(i2cHandle->Instance==I2C2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);
  }
  else if(i2cHandle->Instance==I2C3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
  }
}

