#include "imu_sensor.h"

#include "ist_protocol.h"

#include "bsp_iic.h"

/*----------�ⲿ����----------*/
extern I2C_HandleTypeDef hi2c3;


/*----------IST8310��ʼ����غ���----------*/
/*IST8310�����λ*/
static void IST8310_Reset(void);
/*IST8310���üĴ���*/
static void IST8310_Config_Reg(ist8310_info_t* info);

	
/*----------IST8310��ʼ����غ���----------*/
/*IST8310��ʼ��*/
void IST8310_Init(ist8310_info_t* info)
{	
	/*�����λ*/
	IST8310_Reset();

	/*����IST8310�Ĵ���*/
	do{
		IST8310_Config_Reg(info);
	}while(info->error != Dvc_None_Error && info->offline_cnt < info->offline_max_cnt);
}

/*IST8310�����λ*/
static void IST8310_Reset(void)
{
	/*RSTN�ܽ���0���ô����ƺ�RSTN�ܽ���1*/
    IST8310_RSTN_L();
    IST8310_Delay_Ms(IST8310_SLEEP_TIME);
    IST8310_RSTN_H();
    IST8310_Delay_Ms(IST8310_SLEEP_TIME);
}

/*IST8310���üĴ���*/
static void IST8310_Config_Reg(ist8310_info_t* info)
{
    uint8_t res = 0;
    uint8_t writeNum = 0;

	info->offline_cnt++;
	info->error = Dvc_None_Error;
	
	/*��ʼ��������������*//*��ַ������*/
	  uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][2] =
		{
			/*�����ж�*//*����Ϊ�͵�ƽ����*/
			{0x0B, 0x08},
			/*���ò���Ƶ��*//*���β���ƽ��*/
			{0x41, 0x09},
			/*��Ҫ���ó�0xC0*//*��������δ֪*/
			{0x42, 0xC0},
			/*��������ģʽ200HzƵ�����*/
			{0x0A, 0x0B}};

	/*д�����õ�ģʽ*/
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


/*----------IST8310���ݶ�ȡ��غ���----------*/
/*�����ƶ�ȡ����*//*ͨ��I2C ��ȡ����ֽں���*/
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

/*IST8310DRDY�жϴ�����*/
void IST8310_DRDY_Handler(ist8310_info_t *ist)
{
	/*��ȡ���ݼĴ���*/
	IST8310_Read_Mag(ist);
}


