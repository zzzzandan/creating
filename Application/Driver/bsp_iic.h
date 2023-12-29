#ifndef BSP_IIC__H__
#define BSP_IIC__H__

#include "struct_typedef.h"


#define IST8310_IIC_ADDRESS (0x0E << 1)  	/*IST8310的IIC地址*/
#define IST8310_IIC_READ_MSB (0x80) 		/*IST8310的SPI读取发送第一个bit为1*/


extern void IST8310_IIC_Read_Single_Reg(uint8_t reg, uint8_t *res);
extern void IST8310_IIC_Read_Muli_Reg(uint8_t reg, uint8_t *buf, uint8_t len);

extern void IST8310_IIC_Write_Single_Reg(uint8_t reg, uint8_t data);
extern void IST8310_IIC_Write_Muli_Reg(uint8_t reg, uint8_t *data, uint8_t len);

extern void IST8310_RSTN_H(void);
extern void IST8310_RSTN_L(void);

extern void IST8310_Delay_Ms(uint16_t ms);
extern void IST8310_Delay_Us(uint16_t us);



#endif


