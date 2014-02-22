#ifndef _HEIGHTEKF_H_
#define _HEIGHTEKF_H_

#include "OSConfig.h"

typedef struct{
	float height;
	float velo_z;
}VerticalType;

extern xQueueHandle height2FlightQueue;

void vhEKFTask(void* pvParameters);

#endif
