#include "INSEKF.h"

#include <stdio.h>
#include <arm_math.h>
#include "stm32f4xx.h"

#include "UART.h"
#include "string.h"
#include "uartTask.h"
#include "kalman.h"
#include "axisTrans.h"
#include "diskTask.h"
#include "ledTask.h"
#include "ultraSonic.h"

/***********************macro definition*************************/
#define GPS_DELAY_CNT 8

/***********************global variables*************************/
xQueueHandle INSToFlightConQueue;	//pass the navigation infomation to flight controller
xQueueHandle INS2HeightQueue;

float navParamCur[9];	//current navigation parameters, for flight control
float navParamK[9]={0,0,0,0,0,0,0.1,-0.37,-0.01};	//k time navigation parameters, for GPS data fusion
float x[9] = {0,0,0,0,0,0,0.1,-0.37,-0.01};	//navigation parameters error in time k

double initPos[4];
float Ctrans[9];

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
		 0.000001,0,0,0,0,0,0,0,0
	    ,0,0.000001,0,0,0,0,0,0,0
	    ,0,0,0.000001,0,0,0,0,0,0
		,0,0,0,0.025,0,0,0,0,0
		,0,0,0,0,0.025,0,0,0,0
		,0,0,0,0,0,0.025,0,0,0
		,0,0,0,0,0,0,0.000000004096,0,0
		,0,0,0,0,0,0,0,0.000000004096,0
		,0,0,0,0,0,0,0,0,0.000000004096
};	//used to initialize EKF filter structure

const float iR[9]={
		 2,0,0
		,0,2,0
		,0,0,0.1
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
		a2it->acc[0] 	= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN];
		a2it->acc[1] 	= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+1];
		a2it->acc[2] 	= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+2];
		a2it->q[0] 		= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+3];
		a2it->q[1] 		= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+4];
		a2it->q[2] 		= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+5];
		a2it->q[3] 		= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+6];
		a2it->dt 		= IMU_delay_buffer[(GPS_DELAY_CNT-1)*INS_FRAME_LEN+7];
	}
	else
	{
		a2it->acc[0] 	= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN];
		a2it->acc[1] 	= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+1];
		a2it->acc[2] 	= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+2];
		a2it->q[0] 		= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+3];
		a2it->q[1] 		= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+4];
		a2it->q[2] 		= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+5];
		a2it->q[3] 		= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+6];
		a2it->dt 		= IMU_delay_buffer[(buffer_header-1)*INS_FRAME_LEN+7];
	}
}

/*
 * read the oldest imu data from the ring buffer
 */
__inline void ReadBufferBack(AHRS2INSType *a2it)
{
	a2it->acc[0]   	= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN];
	a2it->acc[1]   	= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+1];
	a2it->acc[2]   	= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+2];
	a2it->q[0]   	= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+3];
	a2it->q[1] 		= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+4];
	a2it->q[2] 		= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+5];
	a2it->q[3] 		= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+6];
	a2it->dt  		= IMU_delay_buffer[(buffer_header)*INS_FRAME_LEN+7];
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
//	VisionDataType vdt;
	u16 vision_validate_cnt=0;
	
	float uw_height;
	VisionDataType vdt;
	
	portBASE_TYPE xstatus;
	
	/**/
	xQueueReceive(AHRSToINSQueue, &a2it, portMAX_DELAY);	//capture an INS frame	 

#ifdef INS_DEBUG	
	/*GPS data is not needed in debug mode*/
	while(1)	//
	{		
		/*receive ins data and fill the IMU_delay_buffer*/
		xQueueReceive(AHRSToINSQueue,&a2it,portMAX_DELAY);
		PutToBuffer(&a2it);
		
		GetUltraSonicMeasure(&uw_height);
		
		if(pdPASS == xQueueReceive(xUartVisionQueue, &vdt, 0))
		{
			if(vision_validate_cnt++ >= 3)
			{
				vision_validate_cnt = 0;
				string_len = sprintf(printf_buffer, "%d %d %d\n",vdt.pos_x,vdt.pos_y,vdt.pos_z);
				UartSend(printf_buffer, string_len);
			}
		}
	}

	initPos[0] = 0.0;
	initPos[1] = 0.0;
	initPos[2] = 0.0;
	initPos[3] = uw_height;
	
	navParamK[0] = 0.0;
	navParamK[1] = 0.0;
	navParamK[2] = 0.0;
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
	/*GPS data is not needed in debug mode*/
	while(vision_validate_cnt < 10)	//
	{		
		/*receive ins data and fill the IMU_delay_buffer*/
		xQueueReceive(AHRSToINSQueue,&a2it,portMAX_DELAY);
		PutToBuffer(&a2it);
		
		GetUltraSonicMeasure(&uw_height, RESULT_CLEAR);
		if(xQueueReceive(xUartVisionQueue, &vdt, 0) == pdPASS)
		{
			vision_validate_cnt ++;
		}
	}
	Quat2dcm(Ctrans, a2it.q);

	initPos[0] = vdt.pos_x*0.001;
	initPos[1] = vdt.pos_y*0.001;
	initPos[2] = vdt.pos_z*0.001;
	initPos[3] = uw_height;
	
	navParamK[0] = 0.0;
	navParamK[1] = 0.0;
	navParamK[2] = 0.0;
	navParamK[3] = 0.0;
	navParamK[4] = 0.0;
	navParamK[5] = 0.0;
	navParamK[6] = 0.1;
	navParamK[7] = -0.37;
	navParamK[8] = -0.01;
	
	x[0]=0.0;
	x[1]=0.0;
	x[2]=0.0;
	x[3]=0.0;
	x[4]=0.0;
	x[5]=0.0;
	x[6]=0.1;
	x[7]=-0.37;
	x[8]=-0.01;
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
	u8 i;	
	u8 acc_bias_stable = 0;	//indicate whether acc bias is stably estimated
	
	ekf_filter filter;	
	float dt;
	
	float measure[3]={0};

	AHRS2INSType cur_a2it;
	AHRS2INSType k_a2it;

	float uw_height;

	VisionDataType vdt={0.0,0.0,0.0};
	PosDataType pdt;	//position message send to flight control task

	/*initial filter*/
	filter=ekf_filter_new(9,3,(float *)iQ,(float *)iR
						,INS_GetA,INS_GetH
						,INS_aFunc,INS_hFunc);
	memcpy(filter->x,x,filter->state_dim*sizeof(float));
	memcpy(filter->P,iP,filter->state_dim*filter->state_dim*sizeof(float));

	GetUltraSonicMeasure(&uw_height, RESULT_CLEAR);
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
					, NULL
					, (void *)(&dt)
					, (void *)(filter->A)
					, NULL);
			
//		xstatus=xQueueReceive(xUartVisionQueue,&vdt,0);	//get measurement data
		GetUltraSonicMeasure(&uw_height, RESULT_CLEAR);
		if(xQueueReceive(xUartVisionQueue,&vdt,0) == pdPASS)
		{
			float meas_Err[3]={0.0};
			VisionDataType ned_vdt;
			ned_vdt.pos_x = vdt.pos_y-initPos[1];
			ned_vdt.pos_y = vdt.pos_x-initPos[0];
			ned_vdt.pos_z = initPos[2] - vdt.pos_z;
			
			measure[0] = (Ctrans[0]*ned_vdt.pos_x + Ctrans[1]*ned_vdt.pos_y + Ctrans[2]*ned_vdt.pos_z)*0.001;
			measure[1] = (Ctrans[3]*ned_vdt.pos_x + Ctrans[4]*ned_vdt.pos_y + Ctrans[5]*ned_vdt.pos_z)*0.001;
			measure[2] = uw_height-initPos[3];
			
			for(i=0;i<3;i++)
			{
				meas_Err[i] = navParamK[i] - measure[i];
			}
			
			EKF_update(filter
						, (void *)meas_Err
						, NULL
						, NULL
						, (void *)(filter->x)
						, NULL);	
			/*
			 *correct navParamCur
			 */
			if(acc_bias_stable)
			{
				for(i=0;i<6;i++)
					navParamCur[i] -= filter->x[i];
				navParamCur[6] = filter->x[6];
				navParamCur[7] = filter->x[7];
				navParamCur[8] = filter->x[8];
			}
			
			/*correct navParameters at time K
			 *reset error state x*/			
			for(i=0;i<6;i++)
			{
				navParamK[i] -= filter->x[i];
				filter->x[i] = 0.0;
			}
			
			navParamK[6] = filter->x[6];
			navParamK[7] = filter->x[7];
			navParamK[8] = filter->x[8];
			
			pdt.posX = navParamK[0];
			pdt.posY = navParamK[1];
			pdt.posZ = navParamK[2];
			
			pdt.veloX = navParamK[3];
			pdt.veloY = navParamK[4];
			pdt.veloZ = navParamK[5];
			
			/*put to queue*/
			xQueueSend(INSToFlightConQueue,&pdt,0);			
			
			/*when acc bias stable, 
			 *calculate current navigation parameters*/
			if(!acc_bias_stable && filter->P[60]<0.007)
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
					cur_a2it.acc[0] -= navParamK[6];
					cur_a2it.acc[1] -= navParamK[7];
					cur_a2it.acc[2] -= navParamK[8];
					
					INS_Update(navParamCur,&cur_a2it);
				}
			}
		}
	}
}

void INS_GetA(float *A,void *para1,void *para2,void *para3)
{
	float *quaternion = (float *)para1;
	float dt=*(float *)para3;
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

void INS_GetH(float *H,void *para1,void *para2)
{
	memset(H,0,108);

	H[0]=1;H[10]=1;H[20]=1;
}

void INS_aFunc(float *x,void *para4,void *para5)
{
	float *A=(float *)para4;
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

void INS_hFunc(float *hx,void *para3,void *para4)
{
	float *x = (float *)para3;

	hx[0]=x[0];
	hx[1]=x[1];
	hx[2]=x[2];
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

