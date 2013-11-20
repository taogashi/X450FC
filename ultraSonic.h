#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "stm32f4xx.h"

void TIM2_Config(void);
void TIM2_IT_Config(void);

u8 GetUltraSonicMeasure(float *dist);

#endif
