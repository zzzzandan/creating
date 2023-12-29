#ifndef IMU_SENSOR__H__
#define IMU_SENSOR__H__

#include "robot_config.h"

#include "struct_typedef.h"

/*陀螺仪数据结构体*/
typedef struct bmi088_gyro_info_struct	{
	fp32	gyro_x;
	fp32	gyro_y;	
	fp32	gyro_z;

    dvc_work_state_e	work_state;
    dvc_error_e			error;	
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;
}bmi088_gyro_info_t;

/*加速度计数据结构体*/
typedef struct bmi088_accel_info_struct	{
	fp32	acc_x;
	fp32	acc_y;
	fp32	acc_z;
		
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;
	
    dvc_work_state_e	work_state;
    dvc_error_e			error;	
}bmi088_accel_info_t;

/*BIM088数据结构体*/
typedef struct bmi088_info_struct	{	
	fp32					tempeture;
	uint32_t 				sensor_time;
	bmi088_gyro_info_t		*gyro;
	bmi088_accel_info_t		*accel;	
}bmi088_info_t;

/*磁力计数据结构体*/
typedef struct ist8310_info_struct	{
	fp32	ist_x;
	fp32	ist_y;
	fp32	ist_z;
		
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;
	
    dvc_work_state_e	work_state;
    dvc_error_e			error;	
}ist8310_info_t;

/*IMU传感器数据结构体*/
typedef struct imu_sensor_info_struct {
	bmi088_info_t	*bim;
	ist8310_info_t	*ist;
} imu_sensor_info_t;

/*IMU传感器对象结构体*/
typedef struct imu_sensor_struct {
	imu_sensor_info_t	*info;
	void				(*init)(struct imu_sensor_struct *self);
	void				(*update)(struct imu_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct imu_sensor_struct *self);	
	void				(*heart_beat)(struct imu_sensor_struct *self);
    dvc_id_t			id;	
} imu_sensor_t;


extern bmi088_gyro_info_t	BIM088_Gyro_Info;
extern bmi088_accel_info_t	BIM088_Accel_Info;
extern bmi088_info_t	BMI088_Info;
extern ist8310_info_t	IST8310_Info;
extern imu_sensor_info_t 	Imu_Sensor_Info;
extern imu_sensor_t Imu_Sensor;


#endif

