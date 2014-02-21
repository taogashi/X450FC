#include "heightEKF.h"
#include "stm32f4xx.h"
#include "AHRSEKF.h"
#include "INSEKF.h"
#include "kalman.h"
#include "string.h"
#include "axisTrans.h"
#include "ultraSonic.h"

xQueueHandle height2FlightQueue;

const float hP[9]={
	2.0, 0.0, 0.0,
	0.0, 2.0, 0.0,
	0.0, 0.0, 0.64,
};

const float hQ[9]={
	0.16, 0.0, 0.0,
	0.0,  0.36,0.0,
	0.0,  0.0, 0.49
};

const float hR[4]={
	2.0, 0.0,
	0.0, 0.16
};

void height_GetA(float *A, void *dt, void *para2, void *para3);
void height_GetH(float *H, void *para1, void *para2);
void height_aFunc(float *x, void *A, void *para5);
void height_hFunc(float *hx, void *x, void *para4);

void vhEKFTask(void* pvParameters)
{
	u8 i=0;
	portTickType lastTick;
	AHRS2INSType a2it;
	VerticalType vt;
	ekf_filter filter;
	float measure[2]={0.0};//height, accz
	float dt;
	float acc_bias[3]={0.0};

	filter = ekf_filter_new(3,2, (float *)hQ, (float *)hR
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
	
	lastTick = xTaskGetTickCount();
	for(;;)
	{
		xQueueReceive(AHRS2HeightQueue, &a2it, portMAX_DELAY);
		dt = (xTaskGetTickCount() - lastTick)*0.0005;
		lastTick = xTaskGetTickCount();
		
		EKF_predict(filter
					,(void *)(&dt)
					,(void *)NULL
					,(void *)NULL
					,(void *)(filter->A)
					,(void *)NULL);
		if(i++ >= 10)
		{
			float Cbn[9];
			i=0;
			Quat2dcm(Cbn, a2it.q);
			xQueueReceive(INS2HeightQueue, acc_bias, 0);
			measure[1] = Cbn[6]*(a2it.acc[0]-acc_bias[0])
						+Cbn[7]*(a2it.acc[1]-acc_bias[1])
						+Cbn[8]*(a2it.acc[2]-acc_bias[2])
						+ GRAVITY;
			GetUltraSonicMeasure(&(measure[0]),RESULT_RESERVE);
			EKF_update(filter
						,measure
						,(void *)NULL
						,(void *)NULL
						,(void *)(filter->x)
						,(void *)NULL);
		}
		vt.height = filter->x[0];
		vt.velo_z = filter->x[1];
		
		xQueueSend(height2FlightQueue, &vt, 0);
	}
}

void height_GetA(float *A, void *dt, void *para2, void *para3)
{
	A[0] = 1.0; A[1] = -*(float *)dt; A[2] = 0.0;
	A[3] = 0.0; A[4] = 1.0; A[5] = *(float *)dt;
	A[6] = 0.0; A[7] = 0.0; A[8] = 1.0;
}

void height_GetH(float *H, void *para1, void *para2)
{
	H[0] = 1.0; H[1] = 0.0; H[2] = 0.0;
	H[3] = 0.0; H[4] = 0.0; H[5] = 1.0;
}

void height_aFunc(float *x, void *A, void *para5)
{
	float *fA = (float *)A;
	float newx[3];
	
	newx[0] = fA[0]*x[0] + fA[1]*x[1] + fA[2]*x[2];
	newx[0] = fA[3]*x[0] + fA[4]*x[1] + fA[5]*x[2];
	newx[0] = fA[6]*x[0] + fA[7]*x[1] + fA[8]*x[2];
		
	x[0] = newx[0];
	x[1] = newx[1];
	x[2] = newx[2];
}

void height_hFunc(float *hx, void *x, void *para4)
{
	hx[0] = *(float *)x;
	hx[1] = *((float *)x + 2);
}
