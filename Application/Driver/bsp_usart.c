#include "bsp_usart.h"
#include "rc_protocol.h"

#include <string.h>

extern UART_HandleTypeDef huart3;

#define USART3_RX_DATA_FRAME_LEN	(18u)	// ����3����֡����
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	// ����3���ջ���������


__WEAK void USART1_txDataHandler(void);
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf);


static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                  								  uint32_t SrcAddress, \
                                                  								  uint32_t DstAddress, \
                                                   								  uint32_t SecondMemAddress, \
                                                  								  uint32_t DataLength);

static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength);

#ifdef PRINTF_DATA_ENABLE
	extern UART_HandleTypeDef huart1;
	/*����һDMA���ͻ�����*/
	uint8_t usart1_dma_txbuf[USART1_TX_BUF_LEN];

/*����һ��ʼ��*/
void USART1_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart1.hdmatx, (uint32_t)usart1_dma_txbuf, (uint32_t)&huart1.Instance->DR, USART1_TX_BUF_LEN);
}
#endif


/*������DMA���ջ�����*/
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];


/*DMA������m0������ϻص�����*/
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	//����ǰĿ���ڴ�����ΪMemory1
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	/*���ô���һ���ݴ���   ��������ң��������*/
	USART3_rxDataHandler(usart3_dma_rxbuf[0]);
}
/*DMA������m1������ϻص�����*/
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	//����ǰĿ���ڴ�����ΪMemory0
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	/*���ô���һ���ݴ���   ��������ң��������*/
	USART3_rxDataHandler(usart3_dma_rxbuf[1]);
}


/*----------USART�жϻص���غ���----------*/
/*�ܻص�����*/
void Bsp_Uart_IRQHandler(UART_HandleTypeDef *huart)
{
    /*�ж��Ƿ�Ϊ�����ж�*/
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/*�����жϻص�����*/
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/*��������жϱ�־λ*/
	__HAL_UART_CLEAR_IDLEFLAG(huart);
		
	/*������*//*ң��������*/
	if(huart == &huart3)
	{
		/*ʧ��DMA*/
		__HAL_DMA_DISABLE(huart->hdmarx);

		/*��������DMA������*/
		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}	

		/*���㻺����*/
		memset(usart3_dma_rxbuf, 0, USART3_RX_BUF_LEN);

		/*����DMA�Ĵ���*/
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}

#ifdef PRINTF_DATA_ENABLE	
	/*����һ*/
	else if(huart == &huart1)
	{
		/*ʧ��DMA*/
		__HAL_DMA_DISABLE(huart->hdmatx);
		
		/*����DMA������*/
		USART1_txDataHandler();

		/*���usart1_dma_txbuf������*/
		memset(usart1_dma_txbuf, 0, USART1_TX_BUF_LEN);
		
		/*����DMA�Ĵ���*/
		__HAL_DMA_ENABLE(huart->hdmatx);		
	}
#endif

}


/*----------���ڳ�ʼ����غ���----------*/
/*��������ʼ��*//*ң��������*/
void USART3_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	/*��������һ�����ж�*/
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	
	/*����DMA����ģʽ*/
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	

	/*����DMA˫����ģʽ*/
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)usart3_dma_rxbuf[0], \
							    (uint32_t)usart3_dma_rxbuf[1], \
							    USART3_RX_DATA_FRAME_LEN);
	
}


/*�������жϿ���DMA˫����ģʽ*/
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

	/*˫����ģʽ�²�֧���ڴ浽�ڴ洫��*/
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/*����UART��DMA������ɻص�����*/
	/*��ǰʹ�õ��ڴ滺�������ڴ� 0 �ص�*/
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/*��ǰʹ�õ��ڴ滺�������ڴ� 1 �ص�*/
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/*����Ƿ���ڻص�����*/
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
		hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
		return HAL_ERROR;
	}
	
	/*��������*/
	__HAL_LOCK(hdma);

	/*DMA��ǰΪ׼������״̬*/
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/*���� DMA ����״̬*/
		hdma->State = HAL_DMA_STATE_BUSY;
		/*��ʼ���������*/
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;
		/*ʹ��˫����ģʽ*/
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
		/*����DMAĿ���ַ*/
		hdma->Instance->M1AR = SecondMemAddress;		
		/*�������ݳ���*/
		hdma->Instance->NDTR = DataLength;		

		/*���赽�洢��*/
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/*����DMAĿ���ַ*/
			hdma->Instance->PAR = DstAddress;

			/*����DMAԴ��ַ*/
			hdma->Instance->M0AR = SrcAddress;
		}
		/*�洢��������*/
		else
		{
			/*����DMAĿ���ַ*/
			hdma->Instance->PAR = SrcAddress;

			/*����DMAԴ��ַ*/
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/*���DMA��TC��־λ*/
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/*ʹ��DMA����*/
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/*���̽���*/
		__HAL_UNLOCK(hdma);	  
		/*���ص�ǰ״̬*/
		status = HAL_BUSY;		
	}
	
	/*���̽���*/
	__HAL_UNLOCK(hdma);
	return status; 	
}

/*����DMA*/
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/*��������*/
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/*���� DMA ����״̬*/
		hdma->State = HAL_DMA_STATE_BUSY;

		/*��ʼ���������*/
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/*���DBMλ*/
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/*�������ݳ���*/
		hdma->Instance->NDTR = DataLength;

		/*�洢��������*/
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/*����DMAĿ���ַ*/
			hdma->Instance->PAR = DstAddress;

			/*����DMAԴ��ַ*/
			hdma->Instance->M0AR = SrcAddress;
		}
		/*���赽�洢��*/
		else
		{
			/*����DMAԴ��ַ*/
			hdma->Instance->PAR = SrcAddress;

			/*����DMAĿ���ַ*/
			hdma->Instance->M0AR = DstAddress;
		}

		/*ʹ��DMA����*/
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/*���̽���*/
		__HAL_UNLOCK(hdma);

		/*���ص�ǰ״̬*/
		status = HAL_BUSY;
	} 
	return status; 	
}


/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] ��Ҫ��Potocol Layer��ʵ�־���� USART3 ����Э��
 */
__WEAK void USART1_txDataHandler(void)
{

}

/**
 *	@brief	[__WEAK] ��Ҫ��Potocol Layer��ʵ�־���� USART3 ����Э��
 */
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf)
{

}


