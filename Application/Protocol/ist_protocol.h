#ifndef IST_PROTOCOL__H__
#define IST_PROTOCOL__H__

#include "struct_typedef.h"

#include "imu_sensor.h"

/*��λת��*//*ת����uT*/
#define MAG_SEN 0.3f 
/*IST8310��Ҫ���õļĴ�����Ŀ*/
#define IST8310_WRITE_REG_NUM 4 
/*IST8310Ӳ����Ӧʱ��*/
#define IST8310_SLEEP_TIME 50

extern void IST8310_Init(ist8310_info_t* info);
extern void IST8310_Read_Mag(ist8310_info_t *ist);

extern void IST8310_DRDY_Handler(ist8310_info_t *ist);

#endif

