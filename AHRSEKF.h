#ifndef _AHRSEKF_H_
#define _AHRSEKF_H_
#include "stm32f4xx.h"
#include "OSConfig.h"
#include "UART.h"

//#define MEASURE_DIM3
#define MEASURE_DIM6
#define INS_FRAME_LEN 	8
#define INDEX_DT 		INS_FRAME_LEN-1

typedef struct
{
	float rollAngle;
	float pitchAngle;
	float yawAngle;
	float rollAngleRate;
	float pitchAngleRate;
	float yawAngleRate;
}AHRSDataType;

typedef struct{
	float q[4];
	float acc[3];
	float dt;
}AHRS2INSType;

extern xQueueHandle AHRSToFlightConQueue;
extern xQueueHandle AHRSToINSQueue;

void MeasureAngle(float *acc,s16 *mag,float *angle,float *refangle,u8 use_ref);//根据加速度和磁场强度计算姿态
/*
 * user define function, varies from model to model
 */
void AHRS_GetA(float *A,void *para1,void *para2,void *para3);
void AHRS_GetH(float *H,void *para1,void *para2);
void AHRS_aFunc(float *q,void *para1,void *para2);
void AHRS_hFunc(float *hx,void *para1,void *para2);

/*------------------------------tasks----------------------------------------*/
void vAEKFAligTask(void* pvParameters);
void vAEKFProcessTask(void* pvParameters);

#endif
