#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "struct_typedef.h"
#include "dma.h"

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


extern void BMI088_Delay_Ms(uint16_t ms);

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern void Bsp_SPI_TxRxCplt_Handler(SPI_HandleTypeDef *hspi);
extern void Bsp_SPI_Error_Handler(SPI_HandleTypeDef *hspi);

extern void Bsp_SPI_DMA_Init(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t Size);
extern void Bsp_SPI_DMA_Buffer_Transform(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t DataLength);


#endif

