#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "ADIS16405.h"
#include "OSConfig.h"

typedef ADIS16405_DataTypeDef SensorDataType;

extern xQueueHandle xSenToAhrsQueue;

//void vSenSD740Read(void* pvParameters);
//void vSenAD453Read(void* pvParameters);
void vSenAHRSRead(void* pvParameters);

#endif
