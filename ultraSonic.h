#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "stm32f4xx.h"

#define RESULT_CLEAR	1
#define RESULT_RESERVE	0

void TIM2_Config(void);
void TIM2_IT_Config(void);

u8 GetUltraSonicMeasure(float *dist, u8 Clear);

#endif
