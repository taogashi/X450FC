#ifndef _UARTTASK_H_
#define _UARTTASK_H_

#include "OSConfig.h"
#include "stm32f4xx.h"

typedef struct
{
	s16 pos_x;
	s16 pos_y;
	s16 pos_z;
	s16 chksm;
}VisionDataType;

extern xQueueHandle xUartGPSQueue;
extern xQueueHandle xUartVisionQueue;
extern xQueueHandle xUartWayPointQueue;

extern xQueueHandle xUartParaQueue;

void vUartRecTask(void *pvParameters);

#endif

