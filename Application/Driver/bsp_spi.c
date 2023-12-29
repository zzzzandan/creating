#include "bsp_spi.h"

#include "spi.h"
#include "gpio.h"

#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;


/*等待硬件响应*/
void BMI088_Delay_Ms(uint16_t ms)
{
    osDelay(ms);
}


/*片选信号线选择加速度计*//*PA4*/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/*片选信号线取消选择加速度计*//*PA4*/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/*片选信号线选择陀螺仪*//*PB0*/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/*片选信号线取消选择陀螺仪*//*PB0*/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}


/*SPI交换数据*//*发送txdata返回rxdata*/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}


/*SPI数据传输完毕回调函数*/
void Bsp_SPI_TxRxCplt_Handler(SPI_HandleTypeDef *hspi)
{
	/*传输完毕取消片选*/
	BMI088_ACCEL_NS_H();
	BMI088_GYRO_NS_H();
}

/*SPI的DMA初始化*/
void Bsp_SPI_DMA_Init(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t Size)
{
	/*检查是否使能SPI的发送和接收DMA*/
	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmarx));
	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmatx));

	/*检查SPI的传输方向参数*/
	assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State != HAL_SPI_STATE_BUSY_RX)
	{
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/*设置交换信息*/
	hspi->ErrorCode	= HAL_SPI_ERROR_NONE;
	hspi->pTxBuffPtr	= (uint8_t *)pTxData;
	hspi->TxXferSize	= Size;
	hspi->TxXferCount = Size;
	hspi->pRxBuffPtr	= (uint8_t *)pRxData;
	hspi->RxXferSize	= Size;
	hspi->RxXferCount = Size;

	/*初始化句柄中未使用的字段为零*/
	hspi->RxISR		= NULL;
	hspi->TxISR		= NULL;

	/*设置 DMA 中止 CPLT 回调*/
	hspi->hdmarx->XferAbortCallback = NULL;

	/*使能接收DMA*/
	HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr,hspi->RxXferCount);

	/*使能接收DMA请求*/
	SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

	/* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
	is performed in DMA reception complete callback  */
	hspi->hdmatx->XferHalfCpltCallback = NULL;
	hspi->hdmatx->XferCpltCallback	 = NULL;
	hspi->hdmatx->XferErrorCallback	 = NULL;
	hspi->hdmatx->XferAbortCallback	 = NULL;

	/*使能发送DMA*/
	HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR,hspi->TxXferCount);

	/*检查SPI是否使能*/
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/*使能SPI外设*/
		__HAL_SPI_ENABLE(hspi);
	}

	/*使能发送DMA请求*/
	SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
}


/*改变SPI的DMA缓存区*/
void Bsp_SPI_DMA_Buffer_Transform(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t DataLength)
{
		/*失能DMA*//*使能状态无法更改DMA配置*/
		__HAL_DMA_DISABLE(hspi->hdmarx);
		__HAL_DMA_DISABLE(hspi->hdmatx);

		/*确保失效*/
		while(hspi->hdmarx->Instance->CR & DMA_SxCR_EN || hspi->hdmatx->Instance->CR & DMA_SxCR_EN)
		{
		    __HAL_DMA_DISABLE(hspi->hdmarx);
		    __HAL_DMA_DISABLE(hspi->hdmatx);
		}

		/*改变接收缓存区*/
		HAL_DMA_Start_IT(hspi->hdmarx,(uint32_t)&hspi->Instance->DR,(uint32_t)pRxData,DataLength);
		/*改变发送缓存区*/
		HAL_DMA_Start_IT(hspi->hdmatx,(uint32_t)pTxData,(uint32_t)&hspi->Instance->DR,DataLength);
}




