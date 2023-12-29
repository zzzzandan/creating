#ifndef IMU_PROTOCOL__H__
#define IMU_PROTOCOL__H__


#include "robot_config.h"
#include "imu_sensor.h"


extern void Imu_Init(imu_sensor_t *self);
extern void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf);


#endif

