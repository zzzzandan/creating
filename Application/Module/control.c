#include "control.h"

/*CAN1不同ID发送数据缓存*/
short CAN1_0X1ff_BUF[4] = { 0 };
short CAN1_0X200_BUF[4] = { 0 };
short CAN1_0X2ff_BUF[4] = { 0 };

/*CAN2不同ID发送数据缓存*/
short CAN2_0X1ff_BUF[4] = { 0 };
short CAN2_0X200_BUF[4] = { 0 };
short CAN2_0X2ff_BUF[4] = { 0 };

/*控制模块函数*/
static void Control_Output(void);
static void Control_Reset(void);
static void Control_Self_Protect(void);

/*控制模块结构体*/
control_t control = {
	.reset = Control_Reset,
	.output  = Control_Output,
	.self_protect = Control_Self_Protect,
};

/*控制模块输出*/
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

/*控制模块重启*/
static void Control_Reset(void)
{
	uint8_t i = 0;
	/*缓存区清零*/
	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}

	/*CAN发送数据*/
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,CAN1_0X200_BUF);
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,CAN2_0X200_BUF);
	
}

/*输出模块自我保护*/
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




