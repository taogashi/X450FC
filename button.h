#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "stm32f4xx.h"

#define BUTTON_BIT_RESET() Bit_RESET==GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)

void Button_Config(void);
void Button_IT_Config(void);

#endif

