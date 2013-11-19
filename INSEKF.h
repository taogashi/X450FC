#ifndef _INSEKF_H_
#define _INSEKF_H_

#include "OSConfig.h"

//#define INS_DEBUG
#define GRAVITY 9.8015

typedef struct
{
	float posX;
	float posY;
	float posZ;
	float veloX;
	float veloY;
	float veloZ;
}PosConDataType;

extern xQueueHandle INSToFlightConQueue;

void INS_GetA(float *A,void *para1,void *para2,void *para3);
void INS_GetH(float *H,void *para1,void *para2);
void INS_aFunc(float *x,void *para4,void *para5);
void INS_hFunc(float *hx,void *para3,void *para4);

void INS_Update(float *navParam, float *IMU_data);

void vINSAligTask(void* pvParameters);
void vIEKFProcessTask(void* pvParameters);

#endif

