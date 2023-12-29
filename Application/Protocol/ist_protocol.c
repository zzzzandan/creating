#include "imu_sensor.h"

#include "ist_protocol.h"

#include "bsp_iic.h"

/*----------外部变量----------*/
extern I2C_HandleTypeDef hi2c3;


/*----------IST8310初始化相关函数----------*/
/*IST8310软件复位*/
static void IST8310_Reset(void);
/*IST8310配置寄存器*/
static void IST8310_Config_Reg(ist8310_info_t* info);

	
/*----------IST8310初始化相关函数----------*/
/*IST8310初始化*/
void IST8310_Init(ist8310_info_t* info)
{	
	/*软件复位*/
	IST8310_Reset();

	/*配置IST8310寄存器*/
	do{
		IST8310_Config_Reg(info);
	}while(info->error != Dvc_None_Error && info->offline_cnt < info->offline_max_cnt);
}

/*IST8310软件复位*/
static void IST8310_Reset(void)
{
	/*RSTN管脚置0重置磁力计后RSTN管脚置1*/
    IST8310_RSTN_L();
    IST8310_Delay_Ms(IST8310_SLEEP_TIME);
    IST8310_RSTN_H();
    IST8310_Delay_Ms(IST8310_SLEEP_TIME);
}

/*IST8310配置寄存器*/
static void IST8310_Config_Reg(ist8310_info_t* info)
{
    uint8_t res = 0;
    uint8_t writeNum = 0;

	info->offline_cnt++;
	info->error = Dvc_None_Error;
	
	/*初始化配置所用数组*//*地址、数据*/
	  uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][2] =
		{
			/*开启中断*//*配置为低电平触发*/
			{0x0B, 0x08},
			/*配置采样频率*//*两次采样平均*/
			{0x41, 0x09},
			/*需要配置成0xC0*//*具体作用未知*/
			{0x42, 0xC0},
			/*连续测量模式200Hz频率输出*/
			{0x0A, 0x0B}};

	/*写入配置的模式*/
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        IST8310_IIC_Write_Single_Reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        IST8310_Delay_Ms(1);
		
        IST8310_IIC_Read_Single_Reg(ist8310_write_reg_data_error[writeNum][0],&res);
        IST8310_Delay_Ms(1);
		
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
        	info->error = Dvc_Init_Error;
        }
    }
}


/*----------IST8310数据读取相关函数----------*/
/*磁力计读取函数*//*通过I2C 读取多个字节函数*/
void IST8310_Read_Mag(ist8310_info_t *ist)
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    IST8310_IIC_Read_Muli_Reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    ist->ist_x = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    ist->ist_y = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    ist->ist_z = MAG_SEN * temp_ist8310_data;

	ist->offline_cnt = 0;
}

/*IST8310DRDY中断处理函数*/
void IST8310_DRDY_Handler(ist8310_info_t *ist)
{
	/*读取数据寄存器*/
	IST8310_Read_Mag(ist);
}


