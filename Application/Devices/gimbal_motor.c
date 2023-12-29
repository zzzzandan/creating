#include "gimbal_motor.h"
#include "can_protocol.h"

#include <stdlib.h>


gimbal_motor_info_t gimbal_yaw, gimbal_pitch;


extern void Gimbal_Motor_Updata(gimbal_motor_t *motor, uint8_t *rxBuf);
extern void Gimbal_Motor_Init(gimbal_motor_t *motor);

static void Gimbal_Motor_Check(gimbal_motor_t *motor);
static void Gimbal_Motor_Heart_Beat(gimbal_motor_t *motor);


drv_can_t	Gimbal_Motor_Driver[Gimbal_Motor_Count] = {
	[Gimbal_Pitch_Motor] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_PITCH,
		.tx_data = CAN_SendSingleData,
	},
	[Gimbal_Yaw_Motor] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_YAW,
		.tx_data = CAN_SendSingleData,
	},
};


gimbal_motor_info_t Gimbal_Motor_Info[Gimbal_Motor_Count] = {
	[Gimbal_Pitch_Motor] = {
		.offline_max_cnt = 32,
	},
	[Gimbal_Yaw_Motor] = {
		.offline_max_cnt = 32,
	},
};


gimbal_motor_t Gimbal_Motor[Gimbal_Motor_Count] = {
	[Gimbal_Pitch_Motor] = {
		.info = &Gimbal_Motor_Info[Gimbal_Pitch_Motor],
		.driver = &Gimbal_Motor_Driver[Gimbal_Pitch_Motor],
		.init = Gimbal_Motor_Init,
		.update = Gimbal_Motor_Updata,
		.check = Gimbal_Motor_Check,
		.heart_beat = Gimbal_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Gimbal_Pitch,
	},
	[Gimbal_Yaw_Motor] = {
		.info = &Gimbal_Motor_Info[Gimbal_Yaw_Motor],
		.driver = &Gimbal_Motor_Driver[Gimbal_Yaw_Motor],
		.init = Gimbal_Motor_Init,
		.update = Gimbal_Motor_Updata,
		.check = Gimbal_Motor_Check,
		.heart_beat = Gimbal_Motor_Heart_Beat,
		.work_state = Dvc_Offline,
		.id = Dvc_Id_Gimbal_Yaw,
	},
};


/*云台电机数据检查函数*/
static void Gimbal_Motor_Check(gimbal_motor_t *self)
{
	int16_t err;
	gimbal_motor_info_t *motor_info = self->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = 1;//ture
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

/*云台电机离线检测函数*/
static void Gimbal_Motor_Heart_Beat(gimbal_motor_t *self)
{
	gimbal_motor_info_t *motor_info = self->info;
	
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


















