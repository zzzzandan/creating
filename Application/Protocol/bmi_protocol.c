#include "bmi_protocol.h"
#include "BMI088reg.h"

#include "bsp_spi.h"
#include "bsp_pwm.h"

#include "pid.h"

#include <stdbool.h>

/*----------�ⲿ��������----------*/
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;


/*----------��������----------*/
/*���ٶȼ�+-3g��Χ*//*������+-2000��ÿ����Χ*/
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


/*----------���ݻ�����----------*/
/*�������ݻ�����*//*��ȡ��������ַ*/
/*0x82 = 10000000(��ȡģʽ) | 00000010(0x02���������ݵ�ַ)*/
uint8_t Gyro_DMA_Tx_Buffer[SPI_DMA_GYRO_LENGHT] = {0x82};
/*0x92 = 10000000(��ȡģʽ) | 00010010(0x12���ٶȼ����ݵ�ַ)*/
uint8_t Accel_DMA_Tx_Buffer[SPI_DMA_ACCEL_LENGHT] = {0x92,0x92};
/*0xA2 = 10000000(��ȡģʽ) | 00100010(0x22�¶����ݵ�ַ)*/
uint8_t Tempture_DMA_Tx_Buffer[SPI_DMA_TEMPTURE_LENGHT] = {0xA2,0xA2};

/*�������ݻ�����*/
uint8_t Gyro_DMA_Rx_Buffer[SPI_DMA_GYRO_LENGHT];
uint8_t Accel_DMA_Rx_Buffer[SPI_DMA_ACCEL_LENGHT];
uint8_t Tempture_DMA_Rx_Buffer[SPI_DMA_TEMPTURE_LENGHT];


/*����״̬��־λ*/
bmi_data_flag_t BMI_Data_Flag = {
	.gyro_ready = false,
	.accel_ready = false,
	.tempture_ready = false,
	.dma_ready = false,
};


/*----------���ٶȼƼĴ���������غ���----------*/
/*���ٶȼƵ���д��������  */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/*���ٶȼƵ��ζ���������*/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x00);           \
        (data) = BMI088_read_write_byte(0x00);  \
        BMI088_ACCEL_NS_H();                    \
    }
/*----------�����ǼĴ���������غ���----------*/
/*�����ǵ���д��������  */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/*�����ǵ��ζ���������  */
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

/*----------BMI088��д�Ĵ���������غ���----------*/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg,volatile uint8_t *return_data);

/*----------BMI088��ʼ����غ���----------*/	
void BMI088_Init(bmi088_info_t *info);
static void BMI088_Reset(void);
static void BMI088_Accel_Init(bmi088_info_t *info);
static void BMI088_Gyro_Init(bmi088_info_t *info);
		
/*----------BMI088�¶ȿ�����غ���----------*/	
void BMI088_Tempture_Control(bmi088_info_t *info, pid_ctrl_t *pid);

/*----------BMI088��ʼ����غ���----------*/
/*BMI088��ʼ������*/
void BMI088_Init(bmi088_info_t *info)
{
	bmi088_accel_info_t *acc = info->accel;
	bmi088_gyro_info_t *gyr = info->gyro;

	/*BMI088�����λ*/
	BMI088_Reset();	
	
	/*�����ǳ�ʼ������*/
	do{
		BMI088_Gyro_Init(info);
	}while(gyr->error != Dvc_None_Error);
	
	/*���ٶȼƳ�ʼ������*/
	do{
		BMI088_Accel_Init(info);
	}while(acc->error != Dvc_None_Error);
	
		//}while(acc->error != Dvc_None_Error && acc->offline_cnt < acc->offline_max_cnt);
	
	/*SPI��DMA��ʼ��*/
	Bsp_SPI_DMA_Init(&hspi1,Accel_DMA_Tx_Buffer,Accel_DMA_Rx_Buffer,SPI_DMA_ACCEL_LENGHT);
		
}

/*BMI088�����λ����*/
static void BMI088_Reset(void){
    /*�ȴ�BMI088��һ���ϵ縴λ*/
    BMI088_Delay_Ms(BMI088_GYRO_RESET_TIME);
	
	/*�������ٶȼ������λ���ȴ�*/
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_Delay_Ms(BMI088_ACCEL_RESET_TIME);
	
    /*���������������λ���ȴ�*/
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_Delay_Ms(BMI088_GYRO_RESET_TIME);
}

/*���ٶȼƳ�ʼ������*/
static void BMI088_Accel_Init(bmi088_info_t *info)
{
	bmi088_accel_info_t *acc = info->accel;
		
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;
	
	acc->offline_cnt++;
	acc->error = Dvc_None_Error;
	
	/*���ٶ����ýṹ��*/
	uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][2] =
	{
		/*ʹ�ܼ��ٶȼ�*//*ÿ����������Ҫʹ�ܣ������޷���ü��ٶȼ�����*/
		{BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},
		/*ѡ����ٶȼƹ���ģʽ*//*SUSPEND_MODE��ͣģʽ���ݲɼ���ͣ*/
		{BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
		/*���ü��ٶȼƲ���*//*800Hz���Ƶ��*/
		{BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
		/*���ü��ٶȼƾ���*//*+-3g*/
		{BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G},
		/*�����ж�����INT1*//*ʹ�ܡ�����ģʽ���͵�ƽ��Ӧ*/
		{BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW},
		/*������δ����ж�*//*����׼������ʱ����*/
		{BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT}
	};
			
	/*д�����õ�ģʽ*/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
    	BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
		BMI088_Delay_Ms(1);
		
		BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
		BMI088_Delay_Ms(1);

		/*��������Ƿ�д��*/
		if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
		{
			acc->error = Dvc_Init_Error;
		}
    }
}

/*�����ǳ�ʼ������*/
static void BMI088_Gyro_Init(bmi088_info_t *info)
{
	bmi088_gyro_info_t *gyr = info->gyro;
	
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;
	
	gyr->offline_cnt++;
	gyr->error = Dvc_None_Error;	
	
	/*���������ýṹ��*/
	uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][2] =
	{
		/*���������Ƿֱ���*/
		{BMI088_GYRO_RANGE, BMI088_GYRO_2000},
		/*�������������Ƶ��*/
		{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
		/*ʹ��ģʽΪ����ģʽ*/
		{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
		/*ʹ���ⲿ�ж�*//*INT3*/
		{BMI088_GYRO_CTRL, BMI088_DRDY_ON},
		/*�����ж�����*//*����������͵�ƽ��Ӧ*/
		{BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW},
		/*�����жϴ�����ʽ*//*����׼������ʱ����*/
		{BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3}
	};
	
	/*д�����õ�ģʽ*/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
		BMI088_Delay_Ms(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
		BMI088_Delay_Ms(1);

		/*��������Ƿ�д��*/
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            gyr->error = Dvc_Init_Error;
        }
    }
}


/*----------�Ĵ�����ȡд���������----------*/
/*����д��Ĵ�������*/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/*���ζ�ȡ�Ĵ�������*/
static void BMI088_read_single_reg(uint8_t reg, volatile uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x00);
}


/*----------BMI�¶Ȳ�������غ���----------*/
void BMI088_Tempture_Control(bmi088_info_t *info, pid_ctrl_t *pid)
{
	pid->fdb = info->tempeture;
	pid->err = pid->target - pid->fdb;
	/*�¶ȵ����趨ֵ��90%������ʼ���*/
	if (pid->fdb < 0.9f * pid->target)
	{
		BSP_BMI088_Tempture_Control(pid->out_max);
	}
	else
	{
		/*��������ʽPID����*/
		PID_Control_Single_Delta(pid);
		BSP_BMI088_Tempture_Control((uint16_t)pid->out);
	}

	/*���߱���*/
	if(BMI088_Info.accel->work_state == Dvc_Offline)
		BSP_BMI088_Tempture_Control(0);
}

/*----------�жϴ�����غ���----------*/
/*���ٶȼ��жϴ�����*/
static bool target = false;	
void BMI088_Accel_INT1_Handler(bmi088_info_t *info)
{
	bmi088_accel_info_t *accel = info->accel;
	
	/*��ȡ���ٶȼ�����*/
	if(target == false && BMI_Data_Flag.dma_ready == true){		
		BMI_Data_Flag.dma_ready = false;
		
		/*Ƭѡ���ٶȼ�*/	
		BMI088_ACCEL_NS_L();
	
		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Accel_DMA_Tx_Buffer,Accel_DMA_Rx_Buffer,SPI_DMA_ACCEL_LENGHT);

		/*�ı����ݱ�־λ*/
		BMI_Data_Flag.accel_ready = true;

		/*������߼���*/
		accel->offline_cnt = 0;	
		/*�л�Ŀ��*/
		target = true;
	}

	/*��ȡ�¶ȴ���������*/
	else if(target == true && BMI_Data_Flag.dma_ready == true){			
		BMI_Data_Flag.dma_ready = false;
		
		/*Ƭѡ���ٶȼ�*/	
		BMI088_ACCEL_NS_L();
		
		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Tempture_DMA_Tx_Buffer,Tempture_DMA_Rx_Buffer,SPI_DMA_TEMPTURE_LENGHT);

		/*�ı����ݱ�־λ*/
		BMI_Data_Flag.tempture_ready = true;
	
		/*������߼���*/
		accel->offline_cnt = 0; 
		/*�л�Ŀ��*/
		target = false;
	}	
}

/*�������жϴ�����*/
void BMI088_Gyro_INT3_Handler(bmi088_info_t *info)
{	
	bmi088_gyro_info_t *gyro = info->gyro;
		
	if(BMI_Data_Flag.dma_ready == true){		
		BMI_Data_Flag.dma_ready = false;
		/*Ƭѡ������*/
		BMI088_GYRO_NS_L();

		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Gyro_DMA_Tx_Buffer,Gyro_DMA_Rx_Buffer,SPI_DMA_GYRO_LENGHT);
		
		/*�ı����ݱ�־λ*/
		BMI_Data_Flag.gyro_ready = true;
	}
	
	/*������߼���*/
	gyro->offline_cnt = 0;	
}


/*DMA���ݽ�����ϴ�����*/
void BMI088_Data_Ready_Handler(bmi088_info_t *info)
{
	bmi088_gyro_info_t *gyro = info->gyro;
	bmi088_accel_info_t *accel = info->accel;

	/*�м����*/
	int16_t bmi088_raw_temp;	
	int16_t accel_raw_temp;
	int16_t temperate_raw_temp;

	/*���ٶȼ����ݾ���*/
	if(BMI_Data_Flag.accel_ready == true){	
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[3]) << 8) | Accel_DMA_Rx_Buffer[2];
		accel->acc_x = accel_raw_temp * BMI088_ACCEL_SEN;
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[5]) << 8) | Accel_DMA_Rx_Buffer[4];
		accel->acc_y = accel_raw_temp * BMI088_ACCEL_SEN;
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[7]) << 8) | Accel_DMA_Rx_Buffer[6];
		accel->acc_z = accel_raw_temp * BMI088_ACCEL_SEN;

		/*״̬λ����*/
		BMI_Data_Flag.accel_ready = false;
	}
	
	/*���������ݾ���*/
	else if(BMI_Data_Flag.gyro_ready == true){
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[2]) << 8) | Gyro_DMA_Rx_Buffer[1];
		gyro->gyro_x = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[4]) << 8) | Gyro_DMA_Rx_Buffer[3];
		gyro->gyro_y = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[6]) << 8) | Gyro_DMA_Rx_Buffer[5];
		gyro->gyro_z = bmi088_raw_temp * BMI088_GYRO_SEN;

		/*״̬λ����*/
		BMI_Data_Flag.gyro_ready = false;
	}

	/*�¶ȼ����ݾ���*/
	else if(BMI_Data_Flag.tempture_ready == true){
		temperate_raw_temp = (int16_t)((Tempture_DMA_Rx_Buffer[2] << 3) | (Tempture_DMA_Rx_Buffer[3] >> 5));

		if (temperate_raw_temp > 1023)
		{
		    temperate_raw_temp -= 2048;
		}

		info->tempeture = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

		/*״̬λ����*/
		BMI_Data_Flag.tempture_ready = false;
	}

	BMI_Data_Flag.dma_ready = true;
}


