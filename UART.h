#ifndef _UART_H_
#define _UART_H_

#include "stm32f4xx.h"

#define USE_USART1 0

#if USE_USART1
void USART1_DMA_Config(void);
void USART1_DMA_IT_Config(void);
#else
void USART2_DMA_Config(void);
void USART2_DMA_IT_Config(void);
#endif

void USART3_Config(void); //¥Æø⁄…Ë÷√
void USART3_IT_Config(void);

u8 UartSend(char* content, u16 num);

#endif
