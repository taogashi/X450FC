#ifndef _INSEKF_H_
#define _INSEKF_H_

#include "OSConfig.h"
#include "AHRSEKF.h"

#define INS_DEBUG
#define GRAVITY 9.8015

typedef struct
{
	float posX;
	float posY;
	float posZ;
	float veloX;
	float veloY;
	float veloZ;
}PosDataType;

extern xQueueHandle INSToFlightConQueue;
extern xQueueHandle INS2HeightQueue;

void INS_GetA(float *A,void *para1,void *para2,void *para3);
void INS_GetH(float *H,void *para1,void *para2);
void INS_aFunc(float *x,void *para4,void *para5);
void INS_hFunc(float *hx,void *para3,void *para4);

void INS_Update(float *navParam, AHRS2INSType *a2it);

void vINSAligTask(void* pvParameters);
void vIEKFProcessTask(void* pvParameters);

#endif

