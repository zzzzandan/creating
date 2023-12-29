#include "imu_sensor.h"

#include "imu_protocol.h"
#include "bmi_protocol.h"
#include "ist_protocol.h"


extern void Imu_Init(imu_sensor_t *self);
extern void Imu_Updata(imu_sensor_t *self, uint8_t *rxBuf);

static void Imu_Check(imu_sensor_t *self);
static void Imu_Heart_Beat(imu_sensor_t *self);

/*----------IMU���߼����غ���----------*/
/*----------BMI������߼��----------*/
/*BMI088���߼�⺯��*/
static void BMI088_Heart_Beat(imu_sensor_info_t *info);
/*���������߼�⺯��*/
static void Gyro_Heart_Beat(bmi088_info_t *info);
/*���ٶȼ����߼�⺯��*/
static void Accel_Heart_Beat(bmi088_info_t *info);
/*----------IST8310������߼�⺯��----------*/
/*IST8310���߼�⺯��*/
static void IST8310_Heart_Beat(imu_sensor_info_t *info);


/*����������*/
bmi088_gyro_info_t BIM088_Gyro_Info = {
	.work_state = Dvc_Offline,
	.offline_cnt = 0,
	.offline_max_cnt = 32,
};
	
/*���ٶȼ�����*/
bmi088_accel_info_t	BIM088_Accel_Info = {
	.work_state = Dvc_Offline,
	.offline_cnt = 0,
	.offline_max_cnt = 32,
};

/*BIM088����*/	
bmi088_info_t BMI088_Info = {
	.tempeture = 0,
	.sensor_time = 0,
	.gyro	= &BIM088_Gyro_Info,
	.accel = &BIM088_Accel_Info,
};

/*IST8310����*/
ist8310_info_t IST8310_Info = {
	.work_state = Dvc_Offline,
	.offline_cnt = 0,
	.offline_max_cnt = 32,
};

/*IMU����*/
imu_sensor_info_t Imu_Sensor_Info = {
	.bim = &BMI088_Info,
	.ist = &IST8310_Info,
};

imu_sensor_t Imu_Sensor = {
	.info = &Imu_Sensor_Info,
	.init = Imu_Init,
	.update = Imu_Updata,
	.check = Imu_Check,
	.heart_beat = Imu_Heart_Beat,
	.id = Dvc_Id_Imu,	
};

/*IMU��麯��*/
static void Imu_Check(imu_sensor_t *self)
{



}


/*----------IMU���߼����غ���----------*/
/*IMU���߼�⺯��*/
static void Imu_Heart_Beat(imu_sensor_t *self)
{
	imu_sensor_info_t *imu_info = self->info;
	
	BMI088_Heart_Beat(imu_info);
	IST8310_Heart_Beat(imu_info);
}

/*BMI088���߼�⺯��*/
static void BMI088_Heart_Beat(imu_sensor_info_t *info)
{
	bmi088_info_t *bmi088 = info->bim;
	
	Gyro_Heart_Beat(bmi088);
	Accel_Heart_Beat(bmi088);
}

/*���������߼�⺯��*/
static void Gyro_Heart_Beat(bmi088_info_t *info)
{
	bmi088_gyro_info_t *gyr = info->gyro;

	gyr->offline_cnt++;
	
	if(gyr->offline_cnt > gyr->offline_max_cnt) 
	{
		gyr->offline_cnt = gyr->offline_max_cnt;
		gyr->work_state = Dvc_Offline;
	}
	else {
		if(gyr->work_state == Dvc_Offline)
			gyr->work_state = Dvc_Online;
	}
}

/*���ٶȼ����߼�⺯��*/
static void Accel_Heart_Beat(bmi088_info_t *info)
{
	bmi088_accel_info_t *acc = info->accel;

	acc->offline_cnt++;
	
	if(acc->offline_cnt > acc->offline_max_cnt) 
	{
		acc->offline_cnt = acc->offline_max_cnt;
		acc->work_state = Dvc_Offline;
	}
	else {
		if(acc->work_state == Dvc_Offline)
			acc->work_state = Dvc_Online;
	}
}


/*----------IST8310������߼�⺯��----------*/
/*IST8310���߼�⺯��*/
static void IST8310_Heart_Beat(imu_sensor_info_t *info)
{
	ist8310_info_t *ist_info = info->ist;
	
	ist_info->offline_cnt++;

	if(ist_info->offline_cnt > ist_info->offline_max_cnt)
	{
		ist_info->offline_cnt = ist_info->offline_max_cnt;
		ist_info->work_state = Dvc_Offline;
	}
	else {
		if(ist_info->work_state == Dvc_Offline)
			ist_info->work_state = Dvc_Online;
	}
}

