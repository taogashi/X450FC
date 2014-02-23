#include "heightEKF.h"
#include "stm32f4xx.h"
#include "AHRSEKF.h"
#include "INSEKF.h"
#include "kalman.h"
#include <string.h>
#include <stdio.h>
#include "axisTrans.h"
#include "ultraSonic.h"

xQueueHandle height2FlightQueue;

const float hP[9]={
	2.0, 0.0, 0.0,
	0.0, 2.0, 0.0,
	0.0, 0.0, 0.64,
};

const float hQ[9]={
	0.0016, 0.0, 0.0,
	0.0,  0.0036,0.0,
	0.0,  0.0, 0.000016
};

const float hR = 0.3;

void height_GetA(float *A, void *dt, void *para2, void *para3);
void height_GetH(float *H, void *para1, void *para2);
void height_aFunc(float *x, void *A, void *para5);
void height_hFunc(float *hx, void *x, void *para4);

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

	filter = ekf_filter_new(3,1, (float *)hQ, (float *)&hR
							, height_GetA, height_GetH
							, height_aFunc, height_hFunc);
	memcpy(filter->P, hP, filter->state_dim*filter->state_dim*sizeof(float));
	filter->x[0] = 0.0;
	filter->x[1] = 0.0;
	filter->x[2] = 0.0;
	
	/*Enable ultrasonic sensor TIMER*/
	TIM2_Config();
	TIM2_IT_Config();
	
	xQueueReceive(AHRS2HeightQueue, &a2it, portMAX_DELAY);
	while(1 != GetUltraSonicMeasure(&measure, RESULT_RESERVE));
	filter->x[0] = measure;
	
	lastTick = xTaskGetTickCount();
	for(;;)
	{
		xQueueReceive(AHRS2HeightQueue, &a2it, portMAX_DELAY);
		dt = (xTaskGetTickCount() - lastTick)*0.0005;
		lastTick = xTaskGetTickCount();
		
		Quat2dcm(Cbn, a2it.q);
		accz = Cbn[6]*a2it.acc[0]
						+Cbn[7]*a2it.acc[1]
						+Cbn[8]*a2it.acc[2]
						+ GRAVITY;
						
		EKF_predict(filter
					,(void *)(&dt)
					,(void *)NULL
					,(void *)NULL
					,(void *)(&accz)
					,(void *)(&dt));
		if(i++ >= 10)
		{
			
			i=0;
			GetUltraSonicMeasure(&measure, RESULT_RESERVE);

			EKF_update(filter
						,&measure
						,(void *)NULL
						,(void *)NULL
						,(void *)(filter->x)
						,(void *)NULL);
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f\r\n"
//									, filter->x[0]
//									, filter->x[1]
//									, filter->x[2]);
//			UartSend(printf_buffer,string_len);
		}
		vt.height = filter->x[0];
		vt.velo_z = filter->x[1];
		
		xQueueSend(height2FlightQueue, &vt, 0);
	}
}

void height_GetA(float *A, void *dt, void *para2, void *para3)
{
	A[0] = 1.0; A[1] = -*(float *)dt; A[2] = 0.0;
	A[3] = 0.0; A[4] = 1.0; A[5] = -*(float *)dt;
	A[6] = 0.0; A[7] = 0.0; A[8] = 1.0;
}

void height_GetH(float *H, void *para1, void *para2)
{
	H[0] = 1.0; 
}

void height_aFunc(float *x, void *accz, void *dt)
{
	float faccz = *(float *)accz;
	float fdt = *(float *)dt;
	
	x[0] -= x[1]*fdt;
	x[1] += (faccz-x[2])*fdt;
}

void height_hFunc(float *hx, void *x, void *para4)
{
	hx[0] = *(float *)x;
}
