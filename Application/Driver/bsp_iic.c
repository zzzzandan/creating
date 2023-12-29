#include "bsp_iic.h"
#include "i2c.h"
#include "gpio.h"

#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c3;

/*IST8310����������*//*reg�Ĵ�����ַ*/
void IST8310_IIC_Read_Single_Reg(uint8_t reg, uint8_t *res)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,res, 1, 100);
}

/*IST8310����д����*/
void IST8310_IIC_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/*IST8310��ζ�ȡ����*/
void IST8310_IIC_Read_Muli_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/*IST8310���д����*/
void IST8310_IIC_Write_Muli_Reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/*��ʱ�ȴ�Ӳ����Ӧ*/
void IST8310_Delay_Ms(uint16_t ms)
{
    osDelay(ms);
}

/*����IST8310��RSTN����ȡ������*/
void IST8310_RSTN_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}

/*����IST8310��RSTN���ſ�������*/
void IST8310_RSTN_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}






