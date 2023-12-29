#include "imu_sensor.h"

#include "imu_protocol.h"
#include "bmi_protocol.h"
#include "ist_protocol.h"

#include "bsp_iic.h"
#include "bsp_spi.h"

#include "gpio.h"
#include "spi.h"
#include "i2c.h"

/*----------IMU��Ҫִ�к���----------*/
/*IMU��ʼ������*/
void Imu_Init(imu_sensor_t *self);
/*IMU���ݸ��º���*/
void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf);


/*----------IMU��Ҫִ�к���----------*/
/*IMU��ʼ������*/
void Imu_Init(imu_sensor_t *self)
{
	bmi088_info_t *bim_info = self->info->bim;
	ist8310_info_t *ist_info = self->info->ist;

	/*�����ʼ��*/
	BMI088_Init(bim_info);
	IST8310_Init(ist_info);		
}

/*IMU���ݸ��º���*/
void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf)
{
	


}


static void Imu_Slove_Cali(imu_sensor_t *imu_sensor)
{
	
	
}

