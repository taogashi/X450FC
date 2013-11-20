#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f4xx.h"

extern s16  tim4IC1Width;
extern s16  tim4IC2Width;
extern s16  tim4IC3Width;
extern s16  tim4IC4Width;
extern s16  tim5IC1Width;
extern s16  tim5IC2Width;
extern s16  tim5IC3Width;
extern s16  tim5IC4Width;


void TIM3_Config(void);

void TIM4_Config(void);
void TIM4_IT_Config(void);
void TIM5_Config(void);
void TIM5_IT_Config(void);

void Disable_Timer_IT(void);

void Enable_Timer_IT(void);

#endif
