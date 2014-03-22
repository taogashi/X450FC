#include "heightEKF.h"
#include "stm32f4xx.h"
#include "AHRSEKF.h"
#include "INSEKF.h"
#include "kalman.h"
#include <string.h>
#include "axisTrans.h"
#include <stdio.h>

xQueueHandle height2FlightQueue;

const float hP[9]={
	2.0, 0.0, 0.0,
	0.0, 4.0, 0.0,
	0.0, 0.0, 0.64,
};

const float hQ[9]={
	0.0032, 0.0, 0.0,
	0.0,  0.00064,0.0,
	0.0,  0.0, 0.00000049
};

const float hR = 2.0;

void height_GetA(float *A, void *para1,void *para2,void *para3,void *para4);
void height_GetH(float *H, void *para1,void *para2,void *para3,void *para4);
void height_aFunc(float *x, void *para1,void *para2,void *para3,void *para4);
void height_hFunc(float *hx, void *para1,void *para2,void *para3,void *para4);

void vhEKFTask(void* pvParameters)
{
//	char printf_buffer[100];
//	u16 string_len;
	
	u8 i=0;
	portTickType lastTick;
	AHRS2INSType a2it;
	VerticalType vt;
	ekf_filter filter;
	
	float Cbn[9];
	float accz;
	float measure=0.0;//height
	float dt;
	
	float heightParam[3]={0.0, 0.0, 0.1};

	filter = ekf_filter_new(3,1, (float *)hQ, (float *)&hR
							, height_GetA, height_GetH
							, height_aFunc, height_hFunc);
	memcpy(filter->P, hP, filter->state_dim*filter->state_dim*sizeof(float));
	filter->x[0] = 0.0;
	filter->x[1] = 0.0;
	filter->x[2] = 0.1;
	
	xQueueReceive(AHRS2HeightQueue, &a2it, portMAX_DELAY);
	
	lastTick = xTaskGetTickCount();
	for(;;)
	{
		xQueueReceive(AHRS2HeightQueue, &a2it, portMAX_DELAY);
		dt = (xTaskGetTickCount() - lastTick)*0.0005;
		lastTick = xTaskGetTickCount();
		
		if(dt < 0.003)
			dt = 0.003;
		
		//update height parameters
		Quat2dcm(Cbn, a2it.q);
		accz = Cbn[6]*a2it.acc[0]
						+Cbn[7]*a2it.acc[1]
						+Cbn[8]*a2it.acc[2]
						+ GRAVITY;		
		if(accz > -GRAVITY && accz < GRAVITY)
		{
			heightParam[0] -= heightParam[1]*dt;
			heightParam[1] += accz*dt;
			
			//update height err parameters
			EKF_predict(filter
						,(void *)(&dt)
						,(void *)NULL
						,(void *)NULL
						,(void *)NULL);
		}
					
		if(i++ >= 10)
		{
			i=0;
			//compute height err measurement
			measure = a2it.height-heightParam[0];

			EKF_update(filter
						,&measure
						,(void *)(filter->x)
						,(void *)NULL
						,(void *)NULL
						,(void *)NULL);
			
			//reset err state
			heightParam[0] += filter->x[0];
			heightParam[1] += filter->x[1];
			heightParam[2] = filter->x[2];
			
			filter->x[0] = 0.0;
			filter->x[1] = 0.0;
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f\r\n"
//									, vt.height
//									, vt.velo_z
//									, filter->x[2]);
//			UartSend(printf_buffer,string_len);
		}
		vt.height = heightParam[0] + filter->x[0];
		vt.velo_z = heightParam[1] + filter->x[1];
		
		xQueueSend(height2FlightQueue, &vt, 0);
	}
}

void height_GetA(float *A, void *para1,void *para2,void *para3,void *para4)
{
	float dt = *(float *)para1;
	A[0] = 1.0; A[1] = -dt; A[2] = 0.0;
	A[3] = 0.0; A[4] = 1.0; A[5] = dt;
	A[6] = 0.0; A[7] = 0.0; A[8] = 1.0;
}

void height_aFunc(float *x, void *para1, void *para2, void *para3, void *para4)
{
	float dt = *(float *)para1;
	
	x[0] -= x[1]*dt;
	x[1] += x[2]*dt;
}

void height_GetH(float *H, void *para1, void *para2, void *para3, void *para4)
{
	H[0] = 1.0; 
}

void height_hFunc(float *hx, void *para1, void *para2, void *para3, void *para4)
{
	hx[0] = *(float *)para1;
}
