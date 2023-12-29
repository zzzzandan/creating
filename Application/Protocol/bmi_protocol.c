#include "bmi_protocol.h"
#include "BMI088reg.h"

#include "bsp_spi.h"
#include "bsp_pwm.h"

#include "pid.h"

#include <stdbool.h>

/*----------外部变量声明----------*/
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;


/*----------精度设置----------*/
/*加速度计+-3g误差范围*//*陀螺仪+-2000度每秒误差范围*/
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


/*----------数据缓存区----------*/
/*发送数据缓存区*//*读取操作及地址*/
/*0x82 = 10000000(读取模式) | 00000010(0x02陀螺仪数据地址)*/
uint8_t Gyro_DMA_Tx_Buffer[SPI_DMA_GYRO_LENGHT] = {0x82};
/*0x92 = 10000000(读取模式) | 00010010(0x12加速度计数据地址)*/
uint8_t Accel_DMA_Tx_Buffer[SPI_DMA_ACCEL_LENGHT] = {0x92,0x92};
/*0xA2 = 10000000(读取模式) | 00100010(0x22温度数据地址)*/
uint8_t Tempture_DMA_Tx_Buffer[SPI_DMA_TEMPTURE_LENGHT] = {0xA2,0xA2};

/*接收数据缓存区*/
uint8_t Gyro_DMA_Rx_Buffer[SPI_DMA_GYRO_LENGHT];
uint8_t Accel_DMA_Rx_Buffer[SPI_DMA_ACCEL_LENGHT];
uint8_t Tempture_DMA_Rx_Buffer[SPI_DMA_TEMPTURE_LENGHT];


/*本地状态标志位*/
bmi_data_flag_t BMI_Data_Flag = {
	.gyro_ready = false,
	.accel_ready = false,
	.tempture_ready = false,
	.dma_ready = false,
};


/*----------加速度计寄存器操作相关函数----------*/
/*加速度计单次写操作函数  */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/*加速度计单次读操作函数*/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x00);           \
        (data) = BMI088_read_write_byte(0x00);  \
        BMI088_ACCEL_NS_H();                    \
    }
/*----------陀螺仪寄存器操作相关函数----------*/
/*陀螺仪单次写操作函数  */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/*陀螺仪单次读操作函数  */
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

/*----------BMI088读写寄存器操作相关函数----------*/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg,volatile uint8_t *return_data);

/*----------BMI088初始化相关函数----------*/	
void BMI088_Init(bmi088_info_t *info);
static void BMI088_Reset(void);
static void BMI088_Accel_Init(bmi088_info_t *info);
static void BMI088_Gyro_Init(bmi088_info_t *info);
		
/*----------BMI088温度控制相关函数----------*/	
void BMI088_Tempture_Control(bmi088_info_t *info, pid_ctrl_t *pid);

/*----------BMI088初始化相关函数----------*/
/*BMI088初始化函数*/
void BMI088_Init(bmi088_info_t *info)
{
	bmi088_accel_info_t *acc = info->accel;
	bmi088_gyro_info_t *gyr = info->gyro;

	/*BMI088软件复位*/
	BMI088_Reset();	
	
	/*陀螺仪初始化函数*/
	do{
		BMI088_Gyro_Init(info);
	}while(gyr->error != Dvc_None_Error);
	
	/*加速度计初始化函数*/
	do{
		BMI088_Accel_Init(info);
	}while(acc->error != Dvc_None_Error);
	
		//}while(acc->error != Dvc_None_Error && acc->offline_cnt < acc->offline_max_cnt);
	
	/*SPI的DMA初始化*/
	Bsp_SPI_DMA_Init(&hspi1,Accel_DMA_Tx_Buffer,Accel_DMA_Rx_Buffer,SPI_DMA_ACCEL_LENGHT);
		
}

/*BMI088软件复位函数*/
static void BMI088_Reset(void){
    /*等待BMI088第一次上电复位*/
    BMI088_Delay_Ms(BMI088_GYRO_RESET_TIME);
	
	/*重启加速度计软件复位并等待*/
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_Delay_Ms(BMI088_ACCEL_RESET_TIME);
	
    /*重启陀螺仪软件复位并等待*/
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_Delay_Ms(BMI088_GYRO_RESET_TIME);
}

/*加速度计初始化函数*/
static void BMI088_Accel_Init(bmi088_info_t *info)
{
	bmi088_accel_info_t *acc = info->accel;
		
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;
	
	acc->offline_cnt++;
	acc->error = Dvc_None_Error;
	
	/*加速度配置结构体*/
	uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][2] =
	{
		/*使能加速度计*//*每次重启后都需要使能，否则无法获得加速度计数据*/
		{BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},
		/*选择加速度计工作模式*//*SUSPEND_MODE暂停模式数据采集暂停*/
		{BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
		/*配置加速度计参数*//*800Hz输出频率*/
		{BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
		/*配置加速度计精度*//*+-3g*/
		{BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G},
		/*配置中断引脚INT1*//*使能、推挽模式、低电平响应*/
		{BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW},
		/*配置如何触发中断*//*数据准备就绪时触发*/
		{BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT}
	};
			
	/*写入配置的模式*/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
    	BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
		BMI088_Delay_Ms(1);
		
		BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
		BMI088_Delay_Ms(1);

		/*检测配置是否写入*/
		if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
		{
			acc->error = Dvc_Init_Error;
		}
    }
}

/*陀螺仪初始化函数*/
static void BMI088_Gyro_Init(bmi088_info_t *info)
{
	bmi088_gyro_info_t *gyr = info->gyro;
	
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;
	
	gyr->offline_cnt++;
	gyr->error = Dvc_None_Error;	
	
	/*陀螺仪配置结构体*/
	uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][2] =
	{
		/*配置陀螺仪分辨率*/
		{BMI088_GYRO_RANGE, BMI088_GYRO_2000},
		/*配置陀螺仪输出频率*/
		{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
		/*使能模式为正常模式*/
		{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
		/*使能外部中断*//*INT3*/
		{BMI088_GYRO_CTRL, BMI088_DRDY_ON},
		/*配置中断引脚*//*推挽输出、低电平响应*/
		{BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW},
		/*配置中断触发方式*//*数据准备就绪时触发*/
		{BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3}
	};
	
	/*写入配置的模式*/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
		BMI088_Delay_Ms(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
		BMI088_Delay_Ms(1);

		/*检测配置是否写入*/
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            gyr->error = Dvc_Init_Error;
        }
    }
}


/*----------寄存器读取写入操作函数----------*/
/*单次写入寄存器操作*/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/*单次读取寄存器操作*/
static void BMI088_read_single_reg(uint8_t reg, volatile uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x00);
}


/*----------BMI温度补补偿相关函数----------*/
void BMI088_Tempture_Control(bmi088_info_t *info, pid_ctrl_t *pid)
{
	pid->fdb = info->tempeture;
	pid->err = pid->target - pid->fdb;
	/*温度低于设定值的90%，最大功率加热*/
	if (pid->fdb < 0.9f * pid->target)
	{
		BSP_BMI088_Tempture_Control(pid->out_max);
	}
	else
	{
		/*采用增量式PID计算*/
		PID_Control_Single_Delta(pid);
		BSP_BMI088_Tempture_Control((uint16_t)pid->out);
	}

	/*离线保护*/
	if(BMI088_Info.accel->work_state == Dvc_Offline)
		BSP_BMI088_Tempture_Control(0);
}

/*----------中断处理相关函数----------*/
/*加速度计中断处理函数*/
static bool target = false;	
void BMI088_Accel_INT1_Handler(bmi088_info_t *info)
{
	bmi088_accel_info_t *accel = info->accel;
	
	/*读取加速度计数据*/
	if(target == false && BMI_Data_Flag.dma_ready == true){		
		BMI_Data_Flag.dma_ready = false;
		
		/*片选加速度计*/	
		BMI088_ACCEL_NS_L();
	
		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Accel_DMA_Tx_Buffer,Accel_DMA_Rx_Buffer,SPI_DMA_ACCEL_LENGHT);

		/*改变数据标志位*/
		BMI_Data_Flag.accel_ready = true;

		/*清除离线记数*/
		accel->offline_cnt = 0;	
		/*切换目标*/
		target = true;
	}

	/*读取温度传感器数据*/
	else if(target == true && BMI_Data_Flag.dma_ready == true){			
		BMI_Data_Flag.dma_ready = false;
		
		/*片选加速度计*/	
		BMI088_ACCEL_NS_L();
		
		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Tempture_DMA_Tx_Buffer,Tempture_DMA_Rx_Buffer,SPI_DMA_TEMPTURE_LENGHT);

		/*改变数据标志位*/
		BMI_Data_Flag.tempture_ready = true;
	
		/*清除离线记数*/
		accel->offline_cnt = 0; 
		/*切换目标*/
		target = false;
	}	
}

/*陀螺仪中断处理函数*/
void BMI088_Gyro_INT3_Handler(bmi088_info_t *info)
{	
	bmi088_gyro_info_t *gyro = info->gyro;
		
	if(BMI_Data_Flag.dma_ready == true){		
		BMI_Data_Flag.dma_ready = false;
		/*片选陀螺仪*/
		BMI088_GYRO_NS_L();

		Bsp_SPI_DMA_Buffer_Transform(&hspi1,Gyro_DMA_Tx_Buffer,Gyro_DMA_Rx_Buffer,SPI_DMA_GYRO_LENGHT);
		
		/*改变数据标志位*/
		BMI_Data_Flag.gyro_ready = true;
	}
	
	/*清除离线记数*/
	gyro->offline_cnt = 0;	
}


/*DMA数据接收完毕处理函数*/
void BMI088_Data_Ready_Handler(bmi088_info_t *info)
{
	bmi088_gyro_info_t *gyro = info->gyro;
	bmi088_accel_info_t *accel = info->accel;

	/*中间变量*/
	int16_t bmi088_raw_temp;	
	int16_t accel_raw_temp;
	int16_t temperate_raw_temp;

	/*加速度计数据就绪*/
	if(BMI_Data_Flag.accel_ready == true){	
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[3]) << 8) | Accel_DMA_Rx_Buffer[2];
		accel->acc_x = accel_raw_temp * BMI088_ACCEL_SEN;
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[5]) << 8) | Accel_DMA_Rx_Buffer[4];
		accel->acc_y = accel_raw_temp * BMI088_ACCEL_SEN;
		accel_raw_temp = (int16_t)((Accel_DMA_Rx_Buffer[7]) << 8) | Accel_DMA_Rx_Buffer[6];
		accel->acc_z = accel_raw_temp * BMI088_ACCEL_SEN;

		/*状态位清零*/
		BMI_Data_Flag.accel_ready = false;
	}
	
	/*陀螺仪数据就绪*/
	else if(BMI_Data_Flag.gyro_ready == true){
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[2]) << 8) | Gyro_DMA_Rx_Buffer[1];
		gyro->gyro_x = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[4]) << 8) | Gyro_DMA_Rx_Buffer[3];
		gyro->gyro_y = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((Gyro_DMA_Rx_Buffer[6]) << 8) | Gyro_DMA_Rx_Buffer[5];
		gyro->gyro_z = bmi088_raw_temp * BMI088_GYRO_SEN;

		/*状态位清零*/
		BMI_Data_Flag.gyro_ready = false;
	}

	/*温度计数据就绪*/
	else if(BMI_Data_Flag.tempture_ready == true){
		temperate_raw_temp = (int16_t)((Tempture_DMA_Rx_Buffer[2] << 3) | (Tempture_DMA_Rx_Buffer[3] >> 5));

		if (temperate_raw_temp > 1023)
		{
		    temperate_raw_temp -= 2048;
		}

		info->tempeture = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

		/*状态位清零*/
		BMI_Data_Flag.tempture_ready = false;
	}

	BMI_Data_Flag.dma_ready = true;
}


