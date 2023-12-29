#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/*����vofa+ ��ӡ���ݲ���*/
void MX_USART1_UART_Init(void);
/*���ڽ���ң��������*/
void MX_USART3_UART_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

