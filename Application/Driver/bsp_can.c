#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig);
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);

__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/*CAN��������֡�ṹ��*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

/*CAN��������֡�ṹ��*/
typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;

/*����֡�Ķ���*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;


/*CAN ��ʶ����������λ��Ĭ������*/
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// ������
	sFilterConfig->FilterMaskIdLow = 0;						// ������
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// ������������FIFO0
	sFilterConfig->FilterBank = 0;							// ���ù�����0
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 0;
}

/*CAN �����жϻص�����*/
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		/*��ȡ����*/
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		/*��������*/
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
	}
	else if(hcan->Instance == CAN2)
	{
		/*��ȡ����*/
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		/*��������*/
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
	}
}

/*CAN1 ��ʼ��*/
void CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	/*����CAN�˲�������*//*��λ��Ĭ������*//*��ȱʧ������CAN�޷�����ʹ��*/
	CAN_Filter_ParamsInit(&sFilterConfig);
	/*�趨CAN1������*/
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	/*ʹ�ܽ����ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	/*����CAN1*/
	HAL_CAN_Start(&hcan1);
}

/*CAN2 ��ʼ��*/
void CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	/*����CAN�˲�������*//*��λ��Ĭ������*//*��ȱʧ������CAN�޷�����ʹ��*/
	CAN_Filter_ParamsInit(&sFilterConfig);
	/*�趨CAN1������*/
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	/*ʹ�ܽ����ж�*/
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	/*����CAN2*/
	HAL_CAN_Start(&hcan2);
}

/*ͨ��CAN��������*/
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(hcan->Instance == CAN1)
	{
		/*�ȴ�CAN���Ϳ�������*/
		while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
		txFrame = &hcan1TxFrame;
	}
	else if(hcan->Instance == CAN2)
	{
		/*�ȴ�CAN���Ϳ�������*/
		while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 );
		txFrame = &hcan2TxFrame;
	}
	else
		return HAL_ERROR;
	
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;

	/*��������*/
	txFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	txFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	txFrame->data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txFrame->data[5] = (uint8_t)((int16_t)dat[2]);
	txFrame->data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txFrame->data[7] = (uint8_t)((int16_t)dat[3]);		

	/*����CAN���ͺ���*//*�ж��Ƿ��ͳɹ�*/
	if(HAL_CAN_AddTxMessage(hcan, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}	
	return HAL_OK;
}


/*ͨ��CAN1��������*/
uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan1, stdId, dat);
}

/*ͨ��CAN2��������*/
uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan2, stdId, dat);
}


/* Callback functions --------------------------------------------------------*/
/**
 *	@brief	��д CAN RxFifo �жϽ��պ���
 *	@note	��stm32f4xx_hal_can.c��������
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* CAN1 �����ж� */
	if(hcan->Instance == CAN1)
	{
		CAN_Rx_Callback(hcan);
	}else
	if(hcan->Instance == CAN2)
	{
		CAN_Rx_Callback(hcan);
	}
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] ��Ҫ��Potocol Layer��ʵ�־���� CAN1 ����Э��
 */
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

/**
 *	@brief	[__WEAK] ��Ҫ��Potocol Layer��ʵ�־���� CAN2 ����Э��
 */
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

