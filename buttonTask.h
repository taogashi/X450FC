#ifndef _BUTTONTASK_H_
#define _BUTTONTASK_H_

#include "stm32f4xx.h"
#include "OSconfig.h"

typedef enum{
	MODE1,
	MODE2,
	MODE3
}SystemRunMode;

extern SystemRunMode SYSMODE;
extern xQueueHandle xButtonQueueISR;
void vButtonEXTIHandler(void* vParameter);

#endif

