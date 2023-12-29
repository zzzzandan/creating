#ifndef BMI_PROTOCOL__H__
#define BMI_PROTOCOL__H__

#include "struct_typedef.h"
#include "imu_sensor.h"

#include "pid.h"

#include <stdbool.h>


#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM  6
#define BMI088_WRITE_GYRO_REG_NUM   6


#define BMI088_GYRO_RESET_TIME      35
#define BMI088_ACCEL_RESET_TIME     2


#define BMI088_ACCEL_RANGE_3G
#define BMI088_GYRO_RANGE_2000


#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f


/*DMA数据传输长度*/
/*陀螺仪第一个字节无法使用*/
#define SPI_DMA_GYRO_LENGHT			7
/*加速度计前两个字节无法使用*/
#define SPI_DMA_ACCEL_LENGHT		8
/*温度前两个字节无法使用*/
#define SPI_DMA_TEMPTURE_LENGHT		4


typedef struct bmi_data_flag {
	bool gyro_ready;
	bool accel_ready;
	bool tempture_ready;
	bool dma_ready;
}bmi_data_flag_t;


extern void BMI088_Init(bmi088_info_t *info);

extern void BMI088_Data_Ready_Handler(bmi088_info_t *info);

extern void BMI088_Accel_INT1_Handler(bmi088_info_t *info);
extern void BMI088_Gyro_INT3_Handler(bmi088_info_t *info);

extern void BMI088_Tempture_Control(bmi088_info_t *info, pid_ctrl_t *pid)
;


#endif

