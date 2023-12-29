#include "device.h"


device_list_t Device_List = {
	.rc_sen = &Rc_Sensor,
	.imu_sen = &Imu_Sensor,
	
	.gimbal_mtr[Gimbal_Pitch_Motor] = &Gimbal_Motor[Gimbal_Pitch_Motor],
	.gimbal_mtr[Gimbal_Yaw_Motor] = &Gimbal_Motor[Gimbal_Yaw_Motor],
	
	.launcher_mtr[Launcher_Frict_L] = &Launcher_Motor[Launcher_Frict_L],
	.launcher_mtr[Launcher_Frict_R] = &Launcher_Motor[Launcher_Frict_R],
	.launcher_mtr[Launcher_Limit] = &Launcher_Motor[Launcher_Limit],
	.launcher_mtr[Launcher_Dial] = &Launcher_Motor[Launcher_Dial],
	
	.chassis_mtr[Chassis_Motor_LF] = &Chassis_Motor[Chassis_Motor_LF],
	.chassis_mtr[Chassis_Motor_RF] = &Chassis_Motor[Chassis_Motor_RF],
	.chassis_mtr[Chassis_Motor_LB] = &Chassis_Motor[Chassis_Motor_LB],
	.chassis_mtr[Chassis_Motor_RB] = &Chassis_Motor[Chassis_Motor_RB],
};


/*执行外设的初始化函数*/
void Devices_Init(void)
{
	Device_List.rc_sen->init(Device_List.rc_sen);
	Device_List.imu_sen->init(Device_List.imu_sen);
	
	Device_List.gimbal_mtr[Gimbal_Yaw_Motor]->init(Device_List.gimbal_mtr[Gimbal_Yaw_Motor]);
	Device_List.gimbal_mtr[Gimbal_Pitch_Motor]->init(Device_List.gimbal_mtr[Gimbal_Pitch_Motor]);

	Device_List.launcher_mtr[Launcher_Frict_L]->init(Device_List.launcher_mtr[Launcher_Frict_L]);
	Device_List.launcher_mtr[Launcher_Frict_R]->init(Device_List.launcher_mtr[Launcher_Frict_R]);	
	Device_List.launcher_mtr[Launcher_Limit]->init(Device_List.launcher_mtr[Launcher_Limit]);	
	Device_List.launcher_mtr[Launcher_Dial]->init(Device_List.launcher_mtr[Launcher_Dial]);

	Device_List.chassis_mtr[Chassis_Motor_LF]->init(Device_List.chassis_mtr[Chassis_Motor_LF]);
	Device_List.chassis_mtr[Chassis_Motor_RF]->init(Device_List.chassis_mtr[Chassis_Motor_RF]);
	Device_List.chassis_mtr[Chassis_Motor_LB]->init(Device_List.chassis_mtr[Chassis_Motor_LB]);
	Device_List.chassis_mtr[Chassis_Motor_RB]->init(Device_List.chassis_mtr[Chassis_Motor_RB]);
}

void Devices_Heart_Beat(void)
{	
	Rc_Sensor.heart_beat(&Rc_Sensor);
	Imu_Sensor.heart_beat(&Imu_Sensor);
	Gimbal_Motor[Gimbal_Yaw_Motor].heart_beat(&Gimbal_Motor[Gimbal_Yaw_Motor]);
	Gimbal_Motor[Gimbal_Pitch_Motor].heart_beat(&Gimbal_Motor[Gimbal_Pitch_Motor]);
	
	Launcher_Motor[Launcher_Frict_L].heart_beat(&Launcher_Motor[Launcher_Frict_L]);
	Launcher_Motor[Launcher_Frict_R].heart_beat(&Launcher_Motor[Launcher_Frict_R]);
	Launcher_Motor[Launcher_Dial].heart_beat(&Launcher_Motor[Launcher_Dial]);
	Launcher_Motor[Launcher_Limit].heart_beat(&Launcher_Motor[Launcher_Limit]);

	Chassis_Motor[Chassis_Motor_LF].heart_beat(&Chassis_Motor[Chassis_Motor_LF]);
	Chassis_Motor[Chassis_Motor_RF].heart_beat(&Chassis_Motor[Chassis_Motor_RF]);
	Chassis_Motor[Chassis_Motor_LB].heart_beat(&Chassis_Motor[Chassis_Motor_LB]);
	Chassis_Motor[Chassis_Motor_RB].heart_beat(&Chassis_Motor[Chassis_Motor_RB]);
}

