#ifndef IST_PROTOCOL__H__
#define IST_PROTOCOL__H__

#include "struct_typedef.h"

#include "imu_sensor.h"

/*单位转换*//*转换成uT*/
#define MAG_SEN 0.3f 
/*IST8310需要设置的寄存器数目*/
#define IST8310_WRITE_REG_NUM 4 
/*IST8310硬件响应时间*/
#define IST8310_SLEEP_TIME 50

extern void IST8310_Init(ist8310_info_t* info);
extern void IST8310_Read_Mag(ist8310_info_t *ist);

extern void IST8310_DRDY_Handler(ist8310_info_t *ist);

#endif

