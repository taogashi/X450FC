#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "AHRS.h"
#include "OSConfig.h"

extern xQueueHandle xSenToAhrsQueue;

void vSenAHRSRead(void* pvParameters);

#endif
