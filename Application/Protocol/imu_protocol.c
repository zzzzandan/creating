#include "imu_sensor.h"

#include "imu_protocol.h"
#include "bmi_protocol.h"
#include "ist_protocol.h"

#include "bsp_iic.h"
#include "bsp_spi.h"

#include "gpio.h"
#include "spi.h"
#include "i2c.h"

/*----------IMU主要执行函数----------*/
/*IMU初始化函数*/
void Imu_Init(imu_sensor_t *self);
/*IMU数据更新函数*/
void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf);


/*----------IMU主要执行函数----------*/
/*IMU初始化函数*/
void Imu_Init(imu_sensor_t *self)
{
	bmi088_info_t *bim_info = self->info->bim;
	ist8310_info_t *ist_info = self->info->ist;

	/*外设初始化*/
	BMI088_Init(bim_info);
	IST8310_Init(ist_info);		
}

/*IMU数据更新函数*/
void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf)
{
	


}


static void Imu_Slove_Cali(imu_sensor_t *imu_sensor)
{
	
	
}

