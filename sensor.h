#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "ADIS16405.h"
#include "OSConfig.h"

typedef struct{
float pool[10];
u8 length;
u8 head;
} GFilterType;

typedef ADIS16405_DataTypeDef SensorDataType;
extern const float GCoef[5][10];

extern xQueueHandle xSenToAhrsQueue;

//void vSenSD740Read(void* pvParameters);
//void vSenAD453Read(void* pvParameters);
void vSenAHRSRead(void* pvParameters);

float GaussianFilter(GFilterType *gft,float newData);

#endif
