#include "launcher_motor.h"
#include "can_protocol.h"

#include <stdlib.h>

extern void Launcher_Motor_Init(launcher_motor_t *motor);
extern void Launcher_Motor_Updata(launcher_motor_t *motor, uint8_t *rxBuf);
static void Launcher_Motor_Check(launcher_motor_t *motor);
static void Launcher_Motor_Heart_Beat(launcher_motor_t *motor);


drv_can_t	Launcher_Motor_Driver[Launcher_Motor_Count] = {	
	[Launcher_Limit] = {
		.type = DRV_CAN1,
		.can_id = LAUNCHER_CAN_ID_FRICT_R,
		.tx_data = CAN_SendSingleData,
	},
	[Launcher_Dial] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_DIAL,
		.tx_data = CAN_SendSingleData,
	},
	[Launcher_Frict_L] = {
		.type = DRV_CAN1,
		.can_id = LAUNCHER_CAN_ID_FRICT_L,
		.tx_data = CAN_SendSingleData,
	},
	[Launcher_Frict_R] = {
		.type = DRV_CAN1,
		.can_id = LAUNCHER_CAN_ID_FRICT_R,
		.tx_data = CAN_SendSingleData,
	},	
};

// 发射电机信息
launcher_motor_info_t 	Launcher_Motor_Info[Launcher_Motor_Count] = {
	{
		.offline_max_cnt = 32,
	},
	{
		.offline_max_cnt = 32,
	},
	{
		.offline_max_cnt = 32,
	},
	{
		.offline_max_cnt = 32,
	},


};

// 发射电机传感器
launcher_motor_t	Launcher_Motor[Launcher_Motor_Count] = {
	[Launcher_Limit] = {
		.info = &Launcher_Motor_Info[Launcher_Limit],
		.driver = &Launcher_Motor_Driver[Launcher_Limit],
		.init = Launcher_Motor_Init,
		.update = Launcher_Motor_Updata,
		.check = Launcher_Motor_Check,
		.heart_beat = Launcher_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Launcher_Limit,
	},
	[Launcher_Dial] = {
		.info = &Launcher_Motor_Info[Launcher_Dial],
		.driver = &Launcher_Motor_Driver[Launcher_Dial],
		.init = Launcher_Motor_Init,
		.update = Launcher_Motor_Updata,
		.check = Launcher_Motor_Check,
		.heart_beat = Launcher_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Launcher_Dial,
	},
	[Launcher_Frict_L] = {
		.info = &Launcher_Motor_Info[Launcher_Frict_L],
		.driver = &Launcher_Motor_Driver[Launcher_Frict_L],
		.init = Launcher_Motor_Init,
		.update = Launcher_Motor_Updata,
		.check = Launcher_Motor_Check,
		.heart_beat = Launcher_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Launcher_Dial,
	},
	[Launcher_Frict_R] = {
		.info = &Launcher_Motor_Info[Launcher_Frict_R],
		.driver = &Launcher_Motor_Driver[Launcher_Frict_R],
		.init = Launcher_Motor_Init,
		.update = Launcher_Motor_Updata,
		.check = Launcher_Motor_Check,
		.heart_beat = Launcher_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Launcher_Dial,
	},
};

static void Launcher_Motor_Check(launcher_motor_t *motor)
{
	int16_t err;
	launcher_motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = 0;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 计算累计角度*/
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8192 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->angle_sum += 8192 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}	
	motor_info->angle_prev = motor_info->angle;		

}


static void Launcher_Motor_Heart_Beat(launcher_motor_t *motor)
{
	launcher_motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = Dvc_Offline;
	}
	else {
		if(motor->work_state == Dvc_Offline)
			motor->work_state = Dvc_Online;
	}
}









