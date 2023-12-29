#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*LED引脚宏定义*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH

#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH

#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH

#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn

/*IMU陀螺仪加速度计片选引脚宏定义*/
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA

/*IMU陀螺仪加速度计中断引脚*//*发送数据时产生中断信号*/
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn

#define INT3_GYRO_Pin GPIO_PIN_5
#define INT3_GYRO_GPIO_Port GPIOC
#define INT3_GYRO_EXTI_IRQn EXTI9_5_IRQn

/*IMU磁力计引脚宏定义*/
#define RSTN_IST8310_Pin GPIO_PIN_6    		/*RSTN引脚拉低后重启IST8310*/
#define RSTN_IST8310_GPIO_Port GPIOG

#define DRDY_IST8310_Pin GPIO_PIN_3			/*DRDY引脚数据准备后置1*/
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn

void MX_GPIO_Init(void);


#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

