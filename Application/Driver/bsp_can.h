#ifndef __BSP_CAN__H_
#define __BSP_CAN__H_

#include "struct_typedef.h"
#include "can.h"


void CAN1_Init(void);
void CAN2_Init(void);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);
uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat);
uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat);


#endif

