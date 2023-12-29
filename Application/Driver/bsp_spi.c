#include "bsp_spi.h"

#include "spi.h"
#include "gpio.h"

#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;


/*�ȴ�Ӳ����Ӧ*/
void BMI088_Delay_Ms(uint16_t ms)
{
    osDelay(ms);
}


/*Ƭѡ�ź���ѡ����ٶȼ�*//*PA4*/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/*Ƭѡ�ź���ȡ��ѡ����ٶȼ�*//*PA4*/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/*Ƭѡ�ź���ѡ��������*//*PB0*/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/*Ƭѡ�ź���ȡ��ѡ��������*//*PB0*/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}


/*SPI��������*//*����txdata����rxdata*/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}


/*SPI���ݴ�����ϻص�����*/
void Bsp_SPI_TxRxCplt_Handler(SPI_HandleTypeDef *hspi)
{
	/*�������ȡ��Ƭѡ*/
	BMI088_ACCEL_NS_H();
	BMI088_GYRO_NS_H();
}

/*SPI��DMA��ʼ��*/
void Bsp_SPI_DMA_Init(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t Size)
{
	/*����Ƿ�ʹ��SPI�ķ��ͺͽ���DMA*/
	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmarx));
	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmatx));

	/*���SPI�Ĵ��䷽�����*/
	assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State != HAL_SPI_STATE_BUSY_RX)
	{
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/*���ý�����Ϣ*/
	hspi->ErrorCode	= HAL_SPI_ERROR_NONE;
	hspi->pTxBuffPtr	= (uint8_t *)pTxData;
	hspi->TxXferSize	= Size;
	hspi->TxXferCount = Size;
	hspi->pRxBuffPtr	= (uint8_t *)pRxData;
	hspi->RxXferSize	= Size;
	hspi->RxXferCount = Size;

	/*��ʼ�������δʹ�õ��ֶ�Ϊ��*/
	hspi->RxISR		= NULL;
	hspi->TxISR		= NULL;

	/*���� DMA ��ֹ CPLT �ص�*/
	hspi->hdmarx->XferAbortCallback = NULL;

	/*ʹ�ܽ���DMA*/
	HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr,hspi->RxXferCount);

	/*ʹ�ܽ���DMA����*/
	SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

	/* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
	is performed in DMA reception complete callback  */
	hspi->hdmatx->XferHalfCpltCallback = NULL;
	hspi->hdmatx->XferCpltCallback	 = NULL;
	hspi->hdmatx->XferErrorCallback	 = NULL;
	hspi->hdmatx->XferAbortCallback	 = NULL;

	/*ʹ�ܷ���DMA*/
	HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR,hspi->TxXferCount);

	/*���SPI�Ƿ�ʹ��*/
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/*ʹ��SPI����*/
		__HAL_SPI_ENABLE(hspi);
	}

	/*ʹ�ܷ���DMA����*/
	SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
}


/*�ı�SPI��DMA������*/
void Bsp_SPI_DMA_Buffer_Transform(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t DataLength)
{
		/*ʧ��DMA*//*ʹ��״̬�޷�����DMA����*/
		__HAL_DMA_DISABLE(hspi->hdmarx);
		__HAL_DMA_DISABLE(hspi->hdmatx);

		/*ȷ��ʧЧ*/
		while(hspi->hdmarx->Instance->CR & DMA_SxCR_EN || hspi->hdmatx->Instance->CR & DMA_SxCR_EN)
		{
		    __HAL_DMA_DISABLE(hspi->hdmarx);
		    __HAL_DMA_DISABLE(hspi->hdmatx);
		}

		/*�ı���ջ�����*/
		HAL_DMA_Start_IT(hspi->hdmarx,(uint32_t)&hspi->Instance->DR,(uint32_t)pRxData,DataLength);
		/*�ı䷢�ͻ�����*/
		HAL_DMA_Start_IT(hspi->hdmatx,(uint32_t)pTxData,(uint32_t)&hspi->Instance->DR,DataLength);
}




