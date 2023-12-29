#include "can_protocol.h"

#include "gimbal_motor.h"
#include "launcher_motor.h"
#include "chassis_motor.h"

#include "bsp_can.h"


/*----------CAN报文解析----------*/
/*从CAN报文中读取电机的位置反馈*/
static uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = ((uint16_t)rxData[0] << 8| rxData[1]);
	return angle;
}
/*从CAN报文中读取电机的转子转速反馈*/
static int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = ((uint16_t)rxData[2] << 8| rxData[3]);
	return speed;
}
/*从CAN报文中读取电机的实际转矩电流反馈*/
static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = ((int16_t)rxData[4] << 8 | rxData[5]);
	return current;
}


/*----------CAN标识符与下标解析----------*/
/*RM3508 CAN标识符*/
static uint32_t RM3508_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else if((drv->can_id - 0x201U) < 8)
		return 0x1FF;
	else
		return 0x2FF;
}
/*RM3508 CAN数据下标*/
static uint8_t RM3508_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}

/*GM6020 CAN标识符*/
static uint32_t GM6020_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x205U) < 4)
		return 0x1FF;
	else
		return 0x2FF;
}
/*GM6020 CAN数据下标*/
static uint8_t GM6020_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x205U)%4;
}

/*RM2006 CAN标识符*/
static uint32_t RM2006_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else
		return 0x1FF;
}
/*RM2006 CAN数据下标*/
static uint8_t RM2006_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}


/*----------云台电机相关----------*/
/*云台电机数据更新*/
void Gimbal_Motor_Updata(gimbal_motor_t *motor, uint8_t *rxBuf)
{
	gimbal_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}
/*云台电机初始化*/
void Gimbal_Motor_Init(gimbal_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = Dvc_Offline;
	
	motor->error = Dvc_None_Error;
	if(motor->id == Dvc_Id_Gimbal_Yaw) {
		drv_can->drv_id = GM6020_GetDrvId(drv_can);
		drv_can->std_id = GM6020_GetStdId(drv_can);
	}
	else if(motor->id == Dvc_Id_Gimbal_Pitch) {
		drv_can->drv_id = GM6020_GetDrvId(drv_can);
		drv_can->std_id = GM6020_GetStdId(drv_can);
	}
	else {
		motor->error = Dvc_Id_Error;
	}
}


/*----------发射电机相关----------*/
/*发射电机初始化*/
void Launcher_Motor_Init(launcher_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = Dvc_Offline;
	
	motor->error = Dvc_None_Error;
	if(motor->id == Dvc_Id_Launcher_Limit) {
		drv_can->drv_id = RM2006_GetDrvId(drv_can);
		drv_can->std_id = RM2006_GetStdId(drv_can);
	}
	else if(motor->id == Dvc_Id_Launcher_Dial) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == Dvc_Id_Launcher_Frict_L) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == Dvc_Id_Launcher_Frict_R) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else {
		motor->error = Dvc_Id_Error;
	}
}
/*发射电机数据更新*/
void Launcher_Motor_Updata(launcher_motor_t *motor, uint8_t *rxBuf)
{
	launcher_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}


/*----------底盘电机相关----------*/
/*底盘电机初始化*/
void Chassis_Motor_Init(chassis_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = Dvc_Offline;
	motor->error = Dvc_None_Error;
	
	if(motor->id == Dvc_Chassis_Motor_LF) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == Dvc_Chassis_Motor_RF) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == Dvc_Chassis_Motor_LB) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == Dvc_Chassis_Motor_RB) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else {
		motor->error = Dvc_Id_Error;
	}
}
/*底盘电机数据更新*/
void Chassis_Motor_Updata(chassis_motor_t *motor, uint8_t *rxBuf)
{
	chassis_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}


/*----------发送数据相关----------*/
/*CAN 发送数据*/
void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
{
	int16_t txArr[4] = {0, 0, 0, 0};
	
	txArr[drv->drv_id] = txData;
	if(drv->type == DRV_CAN1)
		CAN1_SendData(drv->std_id, txArr);
	else if(drv->type == DRV_CAN2)
		CAN2_SendData(drv->std_id, txArr);
}
/*CAN 发送数据缓冲模式*/
void CAN_SendDataBuff(drv_type_e drv_type, uint32_t std_id, int16_t *txBuff)
{
	if(drv_type == DRV_CAN1)
		CAN1_SendData(std_id, txBuff);
	else if(drv_type == DRV_CAN2)
		CAN2_SendData(std_id, txBuff);
}
/*CAN发送函数*/
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
  uint8_t transmit_mailbox = 0;
  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
  assert_param(IS_CAN_RTR(TxMessage->RTR));
  assert_param(IS_CAN_DLC(TxMessage->DLC));
  /* Select one empty transmit mailbox */
  if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
  {
    transmit_mailbox = 0;
  }
  else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
  {
    transmit_mailbox = 1;
  }
  else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
  {
    transmit_mailbox = 2;
  }
  else
  {
    transmit_mailbox = CAN_TxStatus_NoMailBox;
  }

  if (transmit_mailbox != CAN_TxStatus_NoMailBox)
  {
    /* Set up the Id */
    CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
    if (TxMessage->IDE == CAN_Id_Standard)
    {
      assert_param(IS_CAN_STDID(TxMessage->StdId)); //发送ID = 设定ID 
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId << 21) | TxMessage->RTR);
    }
    else
    {
      assert_param(IS_CAN_EXTID(TxMessage->ExtId));
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->ExtId << 3) | TxMessage->IDE | TxMessage->RTR);
    }
    
    /* Set up the DLC */
    TxMessage->DLC &= (uint8_t)0x0000000F;
    CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;

    /* Set up the data field */
    CANx->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)TxMessage->Data[3] << 24) | ((uint32_t)TxMessage->Data[2] << 16) | ((uint32_t)TxMessage->Data[1] << 8) | ((uint32_t)TxMessage->Data[0]));
    CANx->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)TxMessage->Data[7] << 24) | ((uint32_t)TxMessage->Data[6] << 16) | ((uint32_t)TxMessage->Data[5] << 8) | ((uint32_t)TxMessage->Data[4]));
    /* Request transmission */
    CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
  }
  return transmit_mailbox;
}


/*----------接收数据相关----------*/
/*CAN1 接收数据*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	/*Yaw轴电机*/
	if(canId == GIMBAL_CAN_ID_YAW)
	{
		Gimbal_Motor[Gimbal_Yaw_Motor].update(&Gimbal_Motor[Gimbal_Yaw_Motor], rxBuf);
		Gimbal_Motor[Gimbal_Yaw_Motor].check(&Gimbal_Motor[Gimbal_Yaw_Motor]);
		
//		if(Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum >GIMBAL_YAW_CIRCULAR_STEP/2)
//		{
//			Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum-=GIMBAL_YAW_CIRCULAR_STEP;
//		}
//		
//		else if(Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum < -GIMBAL_YAW_CIRCULAR_STEP/2)
//		{
//			Gimbal_Motor[Gimbal_Yaw_Motor].info->angle_sum +=GIMBAL_YAW_CIRCULAR_STEP;
//		}
//		
		gimbal_yaw = Gimbal_Motor[Gimbal_Yaw_Motor].info[Gimbal_Yaw_Motor];
	}
	
	/*Pitch轴电机*/
	else if(canId == GIMBAL_CAN_ID_PITCH)
	{
		Gimbal_Motor[Gimbal_Pitch_Motor].update(&Gimbal_Motor[Gimbal_Pitch_Motor], rxBuf);
		Gimbal_Motor[Gimbal_Pitch_Motor].check(&Gimbal_Motor[Gimbal_Pitch_Motor]);
		
		gimbal_pitch = Gimbal_Motor[Gimbal_Pitch_Motor].info[Gimbal_Pitch_Motor];
	}
	
	/*限位轮*/
	else if(canId == LAUNCHER_CAN_ID_LIMIT)
	{
		Launcher_Motor[Launcher_Limit].update(&Launcher_Motor[Launcher_Limit], rxBuf);
		Launcher_Motor[Launcher_Limit].check(&Launcher_Motor[Launcher_Limit]);
	}
	
	/*左摩擦轮*/
	else if(canId == LAUNCHER_CAN_ID_FRICT_L)
	{
		Launcher_Motor[Launcher_Frict_L].update(&Launcher_Motor[Launcher_Frict_L], rxBuf);
		Launcher_Motor[Launcher_Frict_L].check(&Launcher_Motor[Launcher_Frict_L]);
	}
	
	/*右摩擦轮*/
	else if(canId == LAUNCHER_CAN_ID_FRICT_R)
	{
		Launcher_Motor[Launcher_Frict_R].update(&Launcher_Motor[Launcher_Frict_R], rxBuf);
		Launcher_Motor[Launcher_Frict_R].check(&Launcher_Motor[Launcher_Frict_R]);
	}
}

/*CAN2 接收数据*/
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	/*左前轮*/
	if(canId == CHASSIS_CAN_ID_LF)
	{
		Chassis_Motor[Chassis_Motor_LF].update(&Chassis_Motor[Chassis_Motor_LF], rxBuf);
		Chassis_Motor[Chassis_Motor_LF].check(&Chassis_Motor[Chassis_Motor_LF]);
	}
	
	/*右前轮*/
	else if(canId == CHASSIS_CAN_ID_RF)
	{
		Chassis_Motor[Chassis_Motor_RF].update(&Chassis_Motor[Chassis_Motor_RF], rxBuf);
		Chassis_Motor[Chassis_Motor_RF].check(&Chassis_Motor[Chassis_Motor_RF]);
	}
	
	/*左后轮*/
	else if(canId == CHASSIS_CAN_ID_LB)
	{
		Chassis_Motor[Chassis_Motor_LB].update(&Chassis_Motor[Chassis_Motor_LB], rxBuf);
		Chassis_Motor[Chassis_Motor_LB].check(&Chassis_Motor[Chassis_Motor_LB]);
	}
	
	/*右后轮*/
	else if(canId == CHASSIS_CAN_ID_RB)
	{
		Chassis_Motor[Chassis_Motor_RB].update(&Chassis_Motor[Chassis_Motor_RB], rxBuf);
		Chassis_Motor[Chassis_Motor_RB].check(&Chassis_Motor[Chassis_Motor_RB]);
	}
	
	/*拨弹轮*/
	else if(canId == LAUNCHER_CAN_ID_DIAL)
	{
		Launcher_Motor[Launcher_Dial].update(&Launcher_Motor[Launcher_Dial], rxBuf);
		Launcher_Motor[Launcher_Dial].check(&Launcher_Motor[Launcher_Dial]);
	}
}

