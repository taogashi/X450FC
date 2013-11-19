#ifndef _LED_H_
#define _LED_H_

#include "stm32f4xx.h"
/* PD15 to be toggled */
#define LED_ON()	GPIO_ResetBits(GPIOD, GPIO_Pin_10)
#define LED_OFF()	GPIO_SetBits(GPIOD, GPIO_Pin_10)

void LED_Config(void);

#endif
