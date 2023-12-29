#include "chassis_motor.h"

#include "can_protocol.h"

#include <stdbool.h>
#include <stdlib.h>


extern void Chassis_Motor_Updata(chassis_motor_t *motor, uint8_t *rxBuf);
extern void Chassis_Motor_Init(chassis_motor_t *motor);
static void Chassis_Motor_Check(chassis_motor_t *motor);
static void Chassis_Motor_Heart_Beat(chassis_motor_t *motor);


drv_can_t	Chassis_Motor_Driver[Chassis_Motor_Count] = {
	[Chassis_Motor_LF] = {
		.type = DRV_CAN2,
		.can_id = CHASSIS_CAN_ID_LF,
		.tx_data = CAN_SendSingleData,
	},
	[Chassis_Motor_RF] = {
		.type = DRV_CAN2,
		.can_id = CHASSIS_CAN_ID_LB,
		.tx_data = CAN_SendSingleData,
	},
	[Chassis_Motor_LB] = {
		.type = DRV_CAN2,
		.can_id = CHASSIS_CAN_ID_RF,
		.tx_data = CAN_SendSingleData,
	},
	[Chassis_Motor_RB] = {
		.type = DRV_CAN2,
		.can_id = CHASSIS_CAN_ID_RB,
		.tx_data = CAN_SendSingleData,
	},
};

chassis_motor_info_t Chassis_Motor_Info[Chassis_Motor_Count] = {
	[Chassis_Motor_LF] = {
		.offline_max_cnt = 32,
	},
	[Chassis_Motor_RF] = {
		.offline_max_cnt = 32,
	},
	[Chassis_Motor_LB] = {
		.offline_max_cnt = 32,
	},
	[Chassis_Motor_RB] = {
		.offline_max_cnt = 32,
	},
};

chassis_motor_t Chassis_Motor[Chassis_Motor_Count] = {
	[Chassis_Motor_LF] = {
		.info = &Chassis_Motor_Info[Chassis_Motor_LF],
		.driver = &Chassis_Motor_Driver[Chassis_Motor_LF],
		.init = Chassis_Motor_Init,
		.update = Chassis_Motor_Updata,
		.check = Chassis_Motor_Check,
		.heart_beat = Chassis_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Chassis_Motor_LF,
	},
	[Chassis_Motor_RF] = {
		.info = &Chassis_Motor_Info[Chassis_Motor_RF],
		.driver = &Chassis_Motor_Driver[Chassis_Motor_RF],
		.init = Chassis_Motor_Init,
		.update = Chassis_Motor_Updata,
		.check = Chassis_Motor_Check,
		.heart_beat = Chassis_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Chassis_Motor_RF,
	},
	[Chassis_Motor_LB] = {
		.info = &Chassis_Motor_Info[Chassis_Motor_LB],
		.driver = &Chassis_Motor_Driver[Chassis_Motor_LB],
		.init = Chassis_Motor_Init,
		.update = Chassis_Motor_Updata,
		.check = Chassis_Motor_Check,
		.heart_beat = Chassis_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Chassis_Motor_LB,
	},
	[Chassis_Motor_RB] = {
		.info = &Chassis_Motor_Info[Chassis_Motor_RB],
		.driver = &Chassis_Motor_Driver[Chassis_Motor_RB],
		.init = Chassis_Motor_Init,
		.update = Chassis_Motor_Updata,
		.check = Chassis_Motor_Check,
		.heart_beat = Chassis_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Chassis_Motor_RB,
	},
};

/*云台电机数据检查函数*/
static void Chassis_Motor_Check(chassis_motor_t *self)
{
	int16_t err;
	chassis_motor_info_t *motor_info = self->info;
	
	/*未初始化*/
	if( !motor_info->init_flag )
	{
		/*初始化标志位*/
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/*过零点 计算累计角度*/
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

/*云台电机离线检测函数*/
static void Chassis_Motor_Heart_Beat(chassis_motor_t *self)
{
	chassis_motor_info_t *motor_info = self->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		self->work_state = Dvc_Offline;
	}
	else {
		if(self->work_state == Dvc_Offline)
			self->work_state = Dvc_Online;
	}
}


