#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


extern I2C_HandleTypeDef hi2c2;

//extern I2C_HandleTypeDef hi2c3;


//void MX_I2C2_Init(void);/*暂定为OLED屏幕*/
void MX_I2C3_Init(void);/*磁力计接收*/


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

