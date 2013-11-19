#ifndef _UART_H_
#define _UART_H_

#include "stm32f4xx.h"

void USART2_DMA_Config(void);
void USART2_DMA_IT_Config(void);
void USART3_Config(void); //¥Æø⁄…Ë÷√
void USART3_IT_Config(void);

u8 Uart2Send(char* content, u16 num);

#endif
