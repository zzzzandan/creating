#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "robot_config.h"

#include "rc_sensor.h"
#include "imu_sensor.h"

#include "gimbal_motor.h"
#include "launcher_motor.h"
#include "chassis_motor.h"

typedef struct  {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	gimbal_motor_t		*gimbal_mtr[Gimbal_Motor_Count];
	launcher_motor_t	*launcher_mtr[Launcher_Motor_Count];
	chassis_motor_t		*chassis_mtr[Chassis_Motor_Count];
} device_list_t;


extern device_list_t Device_List;

/* Exported functions --------------------------------------------------------*/
extern void Devices_Init(void);
extern void Devices_Heart_Beat(void);

#endif
