#include "control.h"

/*CAN1��ͬID�������ݻ���*/
short CAN1_0X1ff_BUF[4] = { 0 };
short CAN1_0X200_BUF[4] = { 0 };
short CAN1_0X2ff_BUF[4] = { 0 };

/*CAN2��ͬID�������ݻ���*/
short CAN2_0X1ff_BUF[4] = { 0 };
short CAN2_0X200_BUF[4] = { 0 };
short CAN2_0X2ff_BUF[4] = { 0 };

/*����ģ�麯��*/
static void Control_Output(void);
static void Control_Reset(void);
static void Control_Self_Protect(void);

/*����ģ��ṹ��*/
control_t control = {
	.reset = Control_Reset,
	.output  = Control_Output,
	.self_protect = Control_Self_Protect,
};

/*����ģ�����*/
static void Control_Output(void)
{
	uint8_t i = 0;
		
	CAN_SendDataBuff(DRV_CAN1, 0x200,CAN1_0X200_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,CAN2_0X200_BUF);
		
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,CAN2_0X1ff_BUF);
	
	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
}

/*����ģ������*/
static void Control_Reset(void)
{
	uint8_t i = 0;
	/*����������*/
	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}

	/*CAN��������*/
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,CAN1_0X200_BUF);
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,CAN2_0X200_BUF);
	
}

/*���ģ�����ұ���*/
static void Control_Self_Protect(void)
{
	uint8_t i = 0;

	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,	CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,	CAN2_0X200_BUF);
	
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,	CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,	CAN1_0X200_BUF);
}




