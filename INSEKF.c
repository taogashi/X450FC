#include "INSEKF.h"

#include <stdio.h>
#include <arm_math.h>
#include "stm32f4xx.h"

#include "UART.h"
#include "string.h"
#include "uartTask.h"
#include "kalman.h"
#include "axisTrans.h"
#include "gps.h"
#include "flightConTask.h"

/***********************macro definition*************************/
#define GPS_DELAY_CNT 130

/***********************global variables*************************/
xQueueHandle INSToFlightConQueue;	//pass the navigation infomation to flight controller

float navParamCur[9];	//current navigation parameters, for flight control
float navParamK[9]={0, 0, 0, 0, 0, 0, 0, 0, 0};	//k time navigation parameters, for GPS data fusion
float x[9]={0, 0, 0, 0, 0, 0, 0, 0, 0};	//navigation parameters error in time k

float IMU_delay_buffer[GPS_DELAY_CNT*8];
u8 buffer_header=0;

const float iP[81]={
		 10,0,0,0,0,0,0,0,0
	    ,0,10,0,0,0,0,0,0,0
	    ,0,0,2,0,0,0,0,0,0
		,0,0,0, 5,0,0,0,0,0
		,0,0,0,0, 5,0,0,0,0
		,0,0,0,0,0, 1,0,0,0
		,0,0,0,0,0,0,0.25,0,0
		,0,0,0,0,0,0,0,0.25,0
		,0,0,0,0,0,0,0,0,0.64
};	//used to initialize EKF filter structure

const float iQ[81]={
		 0.0001,0,0,0,0,0,0,0,0
	    ,0,0.0001,0,0,0,0,0,0,0
	    ,0,0,0.0032,0,0,0,0,0,0
		,0,0,0,0.0025,0,0,0,0,0
		,0,0,0,0,0.0025,0,0,0,0
		,0,0,0,0,0,0.00064,0,0,0
		,0,0,0,0,0,0,0.000004096,0,0
		,0,0,0,0,0,0,0,0.000004096,0
		,0,0,0,0,0,0,0,0,0.00000049
};	//used to initialize EKF filter structure

const float iR[25]={
		 10, 0, 0, 0, 0
		,0,  10,0, 0, 0
		,0,  0, 2.0, 0, 0
		,0, 0, 0, 0.25, 0
		,0, 0, 0, 0, 0.25
};	//used to initialize EKF filter structure

/*
 * put new imu data into ring buffer
 * and increase buffer header index
 * 
 */
__inline void PutToBuffer(AHRS2INSType *a2it)
{
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN]   = a2it->acc[0];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+1] = a2it->acc[1];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+2] = a2it->acc[2];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+3] = a2it->q[0];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+4] = a2it->q[1];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+5] = a2it->q[2];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+6] = a2it->q[3];
	IMU_delay_buffer[buffer_header*INS_FRAME_LEN+7] = a2it->dt;
	
	/*buffer_header always points to oldest data*/
	/*thus buffer_header-1 points to latest data*/
	buffer_header++;
	if(buffer_header >= GPS_DELAY_CNT)
		buffer_header=0;	
}

/*
 * read the latest imu data from the ring buffer
 */
__inline void ReadBufferFront(AHRS2INSType *a2it)
{
	if(buffer_header == 0)
	{
		a2it->acc[0] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN];
		a2it->acc[1] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+1];
		a2it->acc[2] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+2];
		a2it->q[0] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+3];
		a2it->q[1] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+4];
		a2it->q[2] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+5];
		a2it->q[3] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+6];
		a2it->dt = IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+7];
	}
	else
	{
		a2it->acc[0] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN];
		a2it->acc[1] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+1];
		a2it->acc[2] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+2];
		a2it->q[0] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+3];
		a2it->q[1] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+4];
		a2it->q[2] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+5];
		a2it->q[3] = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+6];
		a2it->dt = IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+7];
	}
}

/*
 * read the oldest imu data from the ring buffer
 */
__inline void ReadBufferBack(AHRS2INSType *a2it)
{
	a2it->acc[0]   = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN];
	a2it->acc[1]   = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+1];
	a2it->acc[2]   = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+2];
	a2it->q[0]   = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+3];
	a2it->q[1] = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+4];
	a2it->q[2] = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+5];
	a2it->q[3] = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+6];
	a2it->dt  = IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+7];
}

/*
 * read the designate imu data from the ring buffer
 */
__inline void ReadBufferIndex(AHRS2INSType *a2it, u8 index)
{
	a2it->acc[0]   = IMU_delay_buffer[(index)*INS_FRAME_LEN];
	a2it->acc[1]   = IMU_delay_buffer[(index)*INS_FRAME_LEN+1];
	a2it->acc[2]   = IMU_delay_buffer[(index)*INS_FRAME_LEN+2];
	a2it->q[0]   = IMU_delay_buffer[(index)*INS_FRAME_LEN+3];
	a2it->q[1] = IMU_delay_buffer[(index)*INS_FRAME_LEN+4];
	a2it->q[2] = IMU_delay_buffer[(index)*INS_FRAME_LEN+5];
	a2it->q[3] = IMU_delay_buffer[(index)*INS_FRAME_LEN+6];
	a2it->dt  = IMU_delay_buffer[(index)*INS_FRAME_LEN+7];
}

/*
 * fill INS_delay_buffer
 * wait GPS signal
 */
void vINSAligTask(void* pvParameters)
{
	char printf_buffer[100];
	u16 string_len;
	
	/*odometry sensor data*/
	AHRS2INSType a2it;
	
	GPSDataType gdt;

	u16 gps_validate_cnt=0;
	
	float baro_height = 0.0;
	
	portBASE_TYPE xstatus;
	
	xQueueReceive(AHRSToINSQueue, &a2it, portMAX_DELAY);	//capture an INS frame	 

#ifdef INS_DEBUG	
	/*GPS data is not needed in debug mode*/
	while(gps_validate_cnt++ < GPS_DELAY_CNT+10)
	{
		xQueueReceive(AHRSToINSQueue, &a2it, portMAX_DELAY);
		PutToBuffer(&a2it);
		baro_height = 0.98*baro_height + 0.02*a2it.height;
	}
	
	navParamK[0] = 0.0;
	navParamK[1] = 0.0;
	navParamK[2] = baro_height;
	navParamK[3] = 0.0;
	navParamK[4] = 0.0;
	navParamK[5] = 0.0;
	navParamK[6] = 0.0;
	navParamK[7] = 0.0;
	navParamK[8] = 0.0;
	
	x[0]=0.0;
	x[1]=0.0;
	x[2]=0.0;
	x[3]=0.0;
	x[4]=0.0;
	x[5]=0.0;
	x[6]=0.0;
	x[7]=0.0;
	x[8]=0.0;
#else
	//normol mode
	Blinks(LED1, 3);
	/*GPS data is not needed in debug mode*/
	while(gps_validate_cnt < 20)	//
	{		
		/*receive ins data and fill the IMU_delay_buffer*/
		xQueueReceive(AHRSToINSQueue,&a2it,portMAX_DELAY);
		PutToBuffer(&a2it);
		baro_height = 0.98*baro_height + 0.02*a2it.height;
		
		if(pdPASS == xQueueReceive(xUartGPSQueue, &gdt, 0))
		{
			gps_validate_cnt ++;
		}
	}

	GPSSetInitPos(&gdt);
	
	navParamK[0] = 0.0;
	navParamK[1] = 0.0;
	navParamK[2] = baro_height;
	navParamK[3] = 0.0;
	navParamK[4] = 0.0;
	navParamK[5] = 0.0;
	navParamK[6] = 0.0;
	navParamK[7] = 0.0;
	navParamK[8] = 0.0;
	
	x[0]=0.0;
	x[1]=0.0;
	x[2]=0.0;
	x[3]=0.0;
	x[4]=0.0;
	x[5]=0.0;
	x[6]=0.0;
	x[7]=0.0;
	x[8]=0.0;
#endif	
	xstatus=xTaskCreate(vIEKFProcessTask,(signed portCHAR *)"ins_ekf",configMINIMAL_STACK_SIZE+1024,(void *)NULL,tskIDLE_PRIORITY+1,NULL);
	if(xstatus!=pdTRUE)
	{
		string_len = sprintf(printf_buffer, "failed to initialize\r\n");
		UartSend(printf_buffer, string_len);
	}
	vTaskDelete(NULL);
}

/*
 *perform kalman filter at time K, estimate best navigation parameter error at time K
 *estimate navigation parameters at current time when acc bias estimation is stable
 */
void vIEKFProcessTask(void* pvParameters)
{
#ifdef INS_DEBUG
	u8 cnt=0;
#endif
	char printf_buffer[100];
	u16 string_len;
	
	u8 i;
	u8 acc_bias_stable = 0;	//indicate whether acc bias is stably estimated
	
	ekf_filter filter;		
	float measure[5]={0.0};

	AHRS2INSType cur_a2it;
	AHRS2INSType k_a2it;
	float dt;
	
	GPSDataType gdt;
	PosDataType pdt;	//position message send to flight control task

	/*initial filter*/
	filter=ekf_filter_new(9,5,(float *)iQ,(float *)iR,INS_GetA,INS_GetH,INS_aFunc,INS_hFunc);
	
	memcpy(filter->x,x,filter->state_dim*sizeof(float));
	memcpy(filter->P,iP,filter->state_dim*filter->state_dim*sizeof(float));

#ifdef INS_DEBUG
	while(optional_param_global.miscel[4] < 0.0 || optional_param_global.miscel[4]>100.0)
	{
		vTaskDelay((portTickType)(200/portTICK_RATE_MS));
	}
	filter->Q[10] = filter->Q[0] = optional_param_global.miscel[1];
	filter->Q[40] = filter->Q[30] = optional_param_global.miscel[2];
	filter->Q[70] = filter->Q[60] = optional_param_global.miscel[3];
	filter->R[6] = filter->R[0] = optional_param_global.miscel[4];
	filter->R[24] = filter->R[18] = optional_param_global.miscel[5];
#endif
	
	/*capture an INS frame*/
	xQueueReceive(AHRSToINSQueue,&cur_a2it,portMAX_DELAY);
	
	for(;;)
	{
		/*capture an INS frame*/
		xQueueReceive(AHRSToINSQueue,&cur_a2it,portMAX_DELAY);
		PutToBuffer(&cur_a2it);
		
		ReadBufferBack(&k_a2it);
		dt=k_a2it.dt;
		
		/*do INS integration at time K*/
		INS_Update(navParamK, &k_a2it);
		
		/*do INS integration at current time*/
		if(acc_bias_stable)
		{
			INS_Update(navParamCur,&cur_a2it);
		}
		
		/*predict navigation error at time K*/
		EKF_predict(filter
					, (void *)(k_a2it.q)
					, (void *)(&dt)
					, (void *)(filter->A)
					, (void *)NULL);
#ifdef INS_DEBUG
		if(cnt ++ >=150)
		{
			float meas_Err[5]={0.0};
			cnt = 0;
			measure[0] = 0.0;
			measure[1] = 0.0;
			measure[3] = 0.0;
			measure[4] = 0.0;
#else
		if(pdPASS == xQueueReceive(xUartGPSQueue,&gdt,0))
		{
			float meas_Err[5]={0.0};
			GPSGetLocalXY(&gdt, measure, measure+1, measure+3, measure+4, 6.1677);
#endif		
			measure[2] = cur_a2it.height;
			
			for(i=0;i<5;i++)
			{
				meas_Err[i] = measure[i] - navParamK[i];
			}
			
			/*update*/
			EKF_update(filter
						, (void *)meas_Err
						, (void *)(filter->x)
						, NULL
						, NULL
						, NULL);	
			/*
			 *correct navParamCur
			 */
			if(acc_bias_stable)
			{
				for(i=0;i<6;i++)
					navParamCur[i] += filter->x[i];
				navParamCur[6] = filter->x[6];
				navParamCur[7] = filter->x[7];
				navParamCur[8] = filter->x[8];
			}
			
			/*correct navParameters at time K
			 *reset error state x*/			
			for(i=0;i<6;i++)
			{
				navParamK[i] += filter->x[i];
				filter->x[i] = 0.0;
			}			
			navParamK[6] = filter->x[6];
			navParamK[7] = filter->x[7];
			navParamK[8] = filter->x[8];		
			
			/*when acc bias stable, 
			 *calculate current navigation parameters*/
			if(!acc_bias_stable && filter->P[60]<0.0345)
			{
				acc_bias_stable = 1;
				//set init value
				for(i=0;i<filter->state_dim;i++)
				{
					navParamCur[i] = navParamK[i];
				}
				//extrapolate current navigation parameters from time K
				for(i=0;i<GPS_DELAY_CNT;i++) 
				{
					if(i+buffer_header >= GPS_DELAY_CNT)
					{
						ReadBufferIndex(&cur_a2it, i+buffer_header-GPS_DELAY_CNT);
					}
					else
					{
						ReadBufferIndex(&cur_a2it, i+buffer_header);
					}
					cur_a2it.acc[0] += navParamK[6];
					cur_a2it.acc[1] += navParamK[7];
					cur_a2it.acc[2] += navParamK[8];
					
					INS_Update(navParamCur,&cur_a2it);
				}
			}
			string_len = sprintf(printf_buffer, "%.2f %.2f %.2f %.2f %.2f %.2f %.4f\r\n"
								,navParamCur[3],navParamCur[4],navParamCur[5]
								,navParamCur[6],navParamCur[7],navParamCur[8]
								,filter->P[60]);
			UartSend(printf_buffer, string_len);
		}
		if(acc_bias_stable == 1)
		{
			pdt.posX = navParamCur[0]+filter->x[0];
			pdt.posY = navParamCur[1]+filter->x[1];
			pdt.posZ = navParamCur[2]+filter->x[2];
			
			pdt.veloX = navParamCur[3]+filter->x[3];
			pdt.veloY = navParamCur[4]+filter->x[4];
			pdt.veloZ = navParamCur[5]+filter->x[5];
			
			/*put to queue*/
			xQueueSend(INSToFlightConQueue,&pdt,0);	
		}
//		string_len = sprintf(printf_buffer, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.3f\r\n"
//								,k_a2it.acc[0],k_a2it.acc[1],k_a2it.acc[2]
//								,navParamK[0],navParamK[1],navParamK[2]
//								,navParamK[3],navParamK[4],navParamK[5]
//								,navParamK[6],navParamK[7],navParamK[8]
//								,dt);
//		xQueueSend(xDiskLogQueue, printf_buffer, 0);
	}
}

void INS_GetA(float *A,void *para1,void *para2,void *para3,void *para4)
{
	float *quaternion = (float *)para1;
	float dt=*(float *)para2;
	float Cbn[9]={0.0};

	memset(A,0,324);

	A[0]=1.0;	A[10]=1.0;	A[20]=1.0;	A[30]=1.0;	A[40]=1.0;	
	A[50]=1.0;	A[60]=1.0;	A[70]=1.0;	A[80]=1.0;

	A[3]=dt;A[13]=dt;A[23]=-dt;
	
	Quat2dcm(Cbn,quaternion);
	A[33] = Cbn[0]*dt;	A[34] = Cbn[1]*dt;	A[35] = Cbn[2]*dt;
	A[42] = Cbn[3]*dt;	A[43] = Cbn[4]*dt;	A[44] = Cbn[5]*dt;
	A[51] = Cbn[6]*dt;	A[52] = Cbn[7]*dt;	A[53] = Cbn[8]*dt;
}

void INS_aFunc(float *x,void *para1,void *para2,void *para3,void *para4)
{
	float *A=(float *)para3;
	u8 i=0;
	
	arm_matrix_instance_f32 AMat;
	arm_matrix_instance_f32 xMat;
	arm_matrix_instance_f32 AxMat;
	
	AMat.numRows = 9;
	AMat.numCols = 9;
	AMat.pData = A;
	
	xMat.numRows = 9;
	xMat.numCols = 1;
	xMat.pData = x;
	
	AxMat.numRows = 9;
	AxMat.numCols = 1;
	AxMat.pData = pvPortMalloc(36);
	
	arm_mat_mult_f32(&AMat, &xMat, &AxMat);
	for(i=0;i<9;i++)
	{
		x[i] = AxMat.pData[i];
	}
	vPortFree(AxMat.pData);
}

void INS_GetH(float *H,void *para1,void *para2,void *para3,void *para4)
{
	memset(H,0,180);

	H[0]=1;		H[10]=1;	H[20]=1;
	H[30]=1; 	H[40]=1;
}

void INS_hFunc(float *hx,void *para1,void *para2,void *para3,void *para4)
{
	float *x = (float *)para1;

	hx[0]=x[0];
	hx[1]=x[1];
	hx[2]=x[2];
	hx[3]=x[3];
	hx[4]=x[4];
}

void INS_Update(float *navParam, AHRS2INSType *a2it)
{
	float Cbn[9]={0.0};
	float dt = a2it->dt;
	
	navParam[0] += navParam[3]*dt;
	navParam[1] += navParam[4]*dt;
	navParam[2] -= navParam[5]*dt;

	Quat2dcm(Cbn, a2it->q);
	navParam[3] += (Cbn[0]*a2it->acc[0] + Cbn[1]*a2it->acc[1] + Cbn[2]*a2it->acc[2])*dt;
	navParam[4] += (Cbn[3]*a2it->acc[0] + Cbn[4]*a2it->acc[1] + Cbn[5]*a2it->acc[2])*dt;
	navParam[5] += (Cbn[6]*a2it->acc[0] + Cbn[7]*a2it->acc[1] + Cbn[8]*a2it->acc[2] + GRAVITY)*dt;
}

