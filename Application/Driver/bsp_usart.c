#include "bsp_usart.h"
#include "rc_protocol.h"

#include <string.h>

extern UART_HandleTypeDef huart3;

#define USART3_RX_DATA_FRAME_LEN	(18u)	// 串口3数据帧长度
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	// 串口3接收缓冲区长度


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
	/*串口一DMA发送缓存区*/
	uint8_t usart1_dma_txbuf[USART1_TX_BUF_LEN];

/*串口一初始化*/
void USART1_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart1.hdmatx, (uint32_t)usart1_dma_txbuf, (uint32_t)&huart1.Instance->DR, USART1_TX_BUF_LEN);
}
#endif


/*串口三DMA接收缓存区*/
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];


/*DMA缓存区m0接收完毕回调函数*/
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	//将当前目标内存设置为Memory1
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	/*调用串口一数据处理   函数解算遥控器数据*/
	USART3_rxDataHandler(usart3_dma_rxbuf[0]);
}
/*DMA缓存区m1接收完毕回调函数*/
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	//将当前目标内存设置为Memory0
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	/*调用串口一数据处理   函数解算遥控器数据*/
	USART3_rxDataHandler(usart3_dma_rxbuf[1]);
}


/*----------USART中断回调相关函数----------*/
/*总回调函数*/
void Bsp_Uart_IRQHandler(UART_HandleTypeDef *huart)
{
    /*判断是否为空闲中断*/
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/*空闲中断回调函数*/
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/*清除空闲中断标志位*/
	__HAL_UART_CLEAR_IDLEFLAG(huart);
		
	/*串口三*//*遥控器接收*/
	if(huart == &huart3)
	{
		/*失能DMA*/
		__HAL_DMA_DISABLE(huart->hdmarx);

		/*处理来自DMA的数据*/
		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}	

		/*清零缓存区*/
		memset(usart3_dma_rxbuf, 0, USART3_RX_BUF_LEN);

		/*重启DMA的传输*/
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}

#ifdef PRINTF_DATA_ENABLE	
	/*串口一*/
	else if(huart == &huart1)
	{
		/*失能DMA*/
		__HAL_DMA_DISABLE(huart->hdmatx);
		
		/*处理DMA的数据*/
		USART1_txDataHandler();

		/*清除usart1_dma_txbuf的数据*/
		memset(usart1_dma_txbuf, 0, USART1_TX_BUF_LEN);
		
		/*重启DMA的传输*/
		__HAL_DMA_ENABLE(huart->hdmatx);		
	}
#endif

}


/*----------串口初始化相关函数----------*/
/*串口三初始化*//*遥控器接收*/
void USART3_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	/*开启串口一空闲中断*/
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	
	/*开启DMA接收模式*/
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	

	/*开启DMA双缓冲模式*/
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)usart3_dma_rxbuf[0], \
							    (uint32_t)usart3_dma_rxbuf[1], \
							    USART3_RX_DATA_FRAME_LEN);
	
}


/*不开启中断开启DMA双缓冲模式*/
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

	/*双缓冲模式下不支持内存到内存传输*/
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/*设置UART的DMA传输完成回调函数*/
	/*当前使用的内存缓冲区是内存 0 回调*/
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/*当前使用的内存缓冲区是内存 1 回调*/
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/*检查是否存在回调函数*/
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
		hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
		return HAL_ERROR;
	}
	
	/*进程锁定*/
	__HAL_LOCK(hdma);

	/*DMA当前为准备就绪状态*/
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/*更改 DMA 外设状态*/
		hdma->State = HAL_DMA_STATE_BUSY;
		/*初始化错误代码*/
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;
		/*使能双缓冲模式*/
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
		/*配置DMA目标地址*/
		hdma->Instance->M1AR = SecondMemAddress;		
		/*配置数据长度*/
		hdma->Instance->NDTR = DataLength;		

		/*外设到存储器*/
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/*配置DMA目标地址*/
			hdma->Instance->PAR = DstAddress;

			/*配置DMA源地址*/
			hdma->Instance->M0AR = SrcAddress;
		}
		/*存储区到外设*/
		else
		{
			/*配置DMA目标地址*/
			hdma->Instance->PAR = SrcAddress;

			/*配置DMA源地址*/
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/*清除DMA的TC标志位*/
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/*使能DMA外设*/
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/*进程解锁*/
		__HAL_UNLOCK(hdma);	  
		/*返回当前状态*/
		status = HAL_BUSY;		
	}
	
	/*进程解锁*/
	__HAL_UNLOCK(hdma);
	return status; 	
}

/*开启DMA*/
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/*进程锁定*/
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/*更改 DMA 外设状态*/
		hdma->State = HAL_DMA_STATE_BUSY;

		/*初始化错误代码*/
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/*清除DBM位*/
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/*配置数据长度*/
		hdma->Instance->NDTR = DataLength;

		/*存储区到外设*/
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/*配置DMA目标地址*/
			hdma->Instance->PAR = DstAddress;

			/*配置DMA源地址*/
			hdma->Instance->M0AR = SrcAddress;
		}
		/*外设到存储器*/
		else
		{
			/*配置DMA源地址*/
			hdma->Instance->PAR = SrcAddress;

			/*配置DMA目标地址*/
			hdma->Instance->M0AR = DstAddress;
		}

		/*使能DMA外设*/
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/*进程解锁*/
		__HAL_UNLOCK(hdma);

		/*返回当前状态*/
		status = HAL_BUSY;
	} 
	return status; 	
}


/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART3 处理协议
 */
__WEAK void USART1_txDataHandler(void)
{

}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART3 处理协议
 */
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf)
{

}


