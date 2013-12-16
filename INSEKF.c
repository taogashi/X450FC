#include "INSEKF.h"

#include <stdio.h>
#include <arm_math.h>
#include "stm32f4xx.h"

#include "UART.h"
#include "string.h"
#include "uartTask.h"
#include "AHRSEKF.h"
#include "kalman.h"
#include "axisTrans.h"
#include "diskTask.h"
#include "ledTask.h"
#include "ultraSonic.h"

/***********************macro definition*************************/
#define GPS_DELAY_CNT 4

/***********************global variables*************************/
xQueueHandle INSToFlightConQueue;	//pass the navigation infomation to flight controller

float navParamCur[9];	//current navigation parameters, for flight control
float navParamK[9];	//k time navigation parameters, for GPS data fusion
float x[9];	//navigation parameters error in time k

double initPos[3];
float IMU_delay_buffer[GPS_DELAY_CNT*8];
u8 buffer_header=0;

const float iP[81]={
		 100,0,0,0,0,0,0,0,0
	    ,0,100,0,0,0,0,0,0,0
	    ,0,0,144,0,0,0,0,0,0
		,0,0,0, 9,0,0,0,0,0
		,0,0,0,0, 9,0,0,0,0
		,0,0,0,0,0, 9,0,0,0
		,0,0,0,0,0,0,0.25,0,0
		,0,0,0,0,0,0,0,0.25,0
		,0,0,0,0,0,0,0,0,0.64
};	//used to initialize EKF filter structure

const float iQ[81]={
		 0.0000000001,0,0,0,0,0,0,0,0
	    ,0,0.0000000001,0,0,0,0,0,0,0
	    ,0,0,0.0000000001,0,0,0,0,0,0
		,0,0,0,0.0025,0,0,0,0,0
		,0,0,0,0,0.0025,0,0,0,0
		,0,0,0,0,0,0.0025,0,0,0
		,0,0,0,0,0,0,0.000000004096,0,0
		,0,0,0,0,0,0,0,0.000000004096,0
		,0,0,0,0,0,0,0,0,0.000000004096
};	//used to initialize EKF filter structure

const float iR[9]={
		 10,0,0
		,0,10,0
		,0,0,0.1
};	//used to initialize EKF filter structure

/*
 * put new imu data into ring buffer
 * and increase buffer header index
 * 
 */
__inline void PutToBuffer(float *new_IMU_data)
{
	IMU_delay_buffer[buffer_header*8]   = new_IMU_data[0];
	IMU_delay_buffer[buffer_header*8+1] = new_IMU_data[1];
	IMU_delay_buffer[buffer_header*8+2] = new_IMU_data[2];
	IMU_delay_buffer[buffer_header*8+3] = new_IMU_data[3];
	IMU_delay_buffer[buffer_header*8+4] = new_IMU_data[4];
	IMU_delay_buffer[buffer_header*8+5] = new_IMU_data[5];
	IMU_delay_buffer[buffer_header*8+6] = new_IMU_data[6];
	IMU_delay_buffer[buffer_header*8+7] = new_IMU_data[7];
	
	/*buffer_header always points to oldest data*/
	/*thus buffer_header-1 points to latest data*/
	buffer_header++;
	if(buffer_header >= GPS_DELAY_CNT)
		buffer_header=0;	
}

/*
 * read the latest imu data from the ring buffer
 */
__inline void ReadBufferFront(float *IMU_data)
{
	if(buffer_header == 0)
	{
		IMU_data[0] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8];
		IMU_data[1] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+1];
		IMU_data[2] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+2];
		IMU_data[3] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+3];
		IMU_data[4] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+4];
		IMU_data[5] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+5];
		IMU_data[6] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+6];
		IMU_data[7] = IMU_delay_buffer[(GPS_DELAY_CNT-1)*8+7];
	}
	else
	{
		IMU_data[0] = IMU_delay_buffer[(buffer_header-1)*8];
		IMU_data[1] = IMU_delay_buffer[(buffer_header-1)*8+1];
		IMU_data[2] = IMU_delay_buffer[(buffer_header-1)*8+2];
		IMU_data[3] = IMU_delay_buffer[(buffer_header-1)*8+3];
		IMU_data[4] = IMU_delay_buffer[(buffer_header-1)*8+4];
		IMU_data[5] = IMU_delay_buffer[(buffer_header-1)*8+5];
		IMU_data[6] = IMU_delay_buffer[(buffer_header-1)*8+6];
		IMU_data[7] = IMU_delay_buffer[(buffer_header-1)*8+7];
	}
}

/*
 * read the oldest imu data from the ring buffer
 */
__inline void ReadBufferBack(float *IMU_data)
{
	IMU_data[0] = IMU_delay_buffer[(buffer_header)*8];
	IMU_data[1] = IMU_delay_buffer[(buffer_header)*8+1];
	IMU_data[2] = IMU_delay_buffer[(buffer_header)*8+2];
	IMU_data[3] = IMU_delay_buffer[(buffer_header)*8+3];
	IMU_data[4] = IMU_delay_buffer[(buffer_header)*8+4];
	IMU_data[5] = IMU_delay_buffer[(buffer_header)*8+5];
	IMU_data[6] = IMU_delay_buffer[(buffer_header)*8+6];
	IMU_data[7] = IMU_delay_buffer[(buffer_header)*8+7];
}

/*
 * fill INS_delay_buffer
 * wait GPS signal
 */
void vINSAligTask(void* pvParameters)
{
	char printf_buffer[100];
	
	/*odometry sensor data*/
	float *p_insBuffer;
//	VisionDataType vdt;
	u16 vision_validate_cnt=0;
	
	float uw_height;
	
	portBASE_TYPE xstatus;
	
	/*Enable ultrasonic sensor TIMER*/
	TIM2_Config();
	TIM2_IT_Config();
	
	/**/
	xQueueReceive(AHRSToINSQueue,&p_insBuffer,portMAX_DELAY);	//capture an INS frame	 
	p_insBuffer[INDEX_DT]=0.0;	//the last number in buffer represent time interval, not time
	Blinks(LED1,2);

#ifdef INS_DEBUG	
	/*GPS data is not needed in debug mode*/
	while(1)	//vision_validate_cnt < 10
	{		
		/*receive ins data and fill the IMU_delay_buffer*/
		xQueueReceive(AHRSToINSQueue,&p_insBuffer,portMAX_DELAY);
		PutToBuffer(p_insBuffer);
		/*clear time interval*/
		p_insBuffer[INDEX_DT]=0.0;
		
		if(GetUltraSonicMeasure(&uw_height))
		{
			//vision_validate_cnt ++;
			sprintf(printf_buffer,"%.2f\r\n",uw_height);
			UartSend(printf_buffer,strlen(printf_buffer));
		}
	}

	initPos[0] = 0.0;
	initPos[1] = 0.0;
	initPos[2] = uw_height;
	
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
	
	/*wait while GPS signal not stable*/
	while(vision_validate_cnt<=100)
	{
		xstatus = xQueueReceive(xUartVisionQueue,&vdt,0);
		if(xstatus == pdPASS)
		{
			vision_validate_cnt ++;
		}
		GetUltraSonicMeasure(&uw_height);
		
		/*receive ins data and fill the IMU_delay_buffer*/
		xQueueReceive(AHRSToINSQueue,&p_insBuffer,portMAX_DELAY);
		PutToBuffer(p_insBuffer);
		/*clear time interval*/
		p_insBuffer[INDEX_DT]=0.0;
	}

	/************initialize navParamK*********************/
	initPos[0] = 0.0;
	initPos[1] = 0.0;
	initPos[2] = uw_height;
	
	navParamK[0] = 0.0;
	navParamK[1] = 0.0;
	navParamK[2] = 0.0;
	navParamK[3] = gdt.speedN;
	navParamK[4] = gdt.speedE;
	navParamK[5] = 0.0;
	navParamK[6] = 0.0;
	navParamK[7] = 0.0;
	navParamK[8] = 0.0;
	
	/*initialize filter state param x*/
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
		sprintf(printf_buffer, "failed to initialize\r\n");
		UartSend(printf_buffer, strlen(printf_buffer));
	}
	vTaskDelete(NULL);
}

/*
 *perform kalman filter at time K, estimate best navigation parameter error at time K
 *estimate navigation parameters at current time when acc bias estimation is stable
 */
void vIEKFProcessTask(void* pvParameters)
{
	u8 i,j;
	u8 acc_bias_stable = 0;	//indicate whether acc bias is stably estimated
	char logData[100]={0};
	
	ekf_filter filter;	
	float dt;
	
	float measure[3]={0};
	float insBufferK[INS_FRAME_LEN];
	float insBufferCur[INS_FRAME_LEN];
	float *p_insBuffer;

	float uw_height;

	VisionDataType vdt={0.0,0.0,0.0};
//	portBASE_TYPE xstatus;
	
	PosConDataType pcdt;	//position message send to flight control task

	/*initial filter*/
	filter=ekf_filter_new(9,3,(float *)iQ,(float *)iR,INS_GetA,INS_GetH,INS_aFunc,INS_hFunc);
	memcpy(filter->x,x,filter->state_dim*sizeof(float));
	memcpy(filter->P,iP,filter->state_dim*filter->state_dim*sizeof(float));

	GetUltraSonicMeasure(&uw_height);
	/*capture an INS frame*/
	xQueueReceive(AHRSToINSQueue,&p_insBuffer,portMAX_DELAY);
	/*last number in buffer represent time interval, not time */
	p_insBuffer[INDEX_DT]=0.0;
	
	Blinks(LED1,4);
	
	for(;;)
	{
		/*capture an INS frame*/
		xQueueReceive(AHRSToINSQueue,&p_insBuffer,portMAX_DELAY);
		PutToBuffer(p_insBuffer);
		p_insBuffer[INDEX_DT]=0.0;
		
		ReadBufferBack(insBufferK);
		dt=insBufferK[INDEX_DT];
		
		/*do INS integration at time K*/
		INS_Update(navParamK, insBufferK);
		
		/*do INS integration at current time*/
		if(acc_bias_stable)
		{
			ReadBufferFront(insBufferCur);
			INS_Update(navParamCur,insBufferCur);
		}
		
		/*predict navigation error at time K*/
		EKF_predict(filter
					, (void *)(insBufferK+3)
					, NULL
					, (void *)(&dt)
					, (void *)(filter->A)
					, NULL);
			
//		xstatus=xQueueReceive(xUartVisionQueue,&vdt,0);	//get measurement data
		if(GetUltraSonicMeasure(&uw_height))
		{
			float meas_Err[3]={0.0};
			
			
			measure[0] = vdt.pos_x;
			measure[1] = vdt.pos_y;
			measure[2] = uw_height-initPos[2];
			
			for(i=0;i<3;i++)
			{
				meas_Err[i] = navParamK[i] - measure[i];
			}
			
			/*data record*/
			/*uncomment this to record data for matlab simulation*/
//			sprintf(logData,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
//											insBufferK[0],insBufferK[1],insBufferK[2],
//											insBufferK[3],insBufferK[4],insBufferK[5],
//											insBufferK[6],insBufferK[7],
//											measure[0],measure[1],measure[2],measure[3],measure[4],
//											navParamK[3],navParamK[4]);
//			xQueueSend(xDiskLogQueue,logData,0);
			
//			sprintf(logData, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
//							measure[0],measure[1],measure[3],measure[4],
//							navParamK[0],navParamK[1],navParamK[3],navParamK[4],
//							navParamCur[0],navParamCur[1],navParamCur[3],navParamCur[4]);
//			xQueueSend(xDiskLogQueue,logData,0);
			/*update*/
			EKF_update(filter
						, (void *)meas_Err
						, NULL
						, NULL
						, (void *)(filter->x)
						, NULL);		
//			printf("%.2f %.2f %.4f %.4f %.4f\r\n",meas_Err[0],meas_Err[1],filter->x[6],filter->x[7],filter->x[8]);
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
				
				pcdt.posX = navParamCur[0];
				pcdt.posY = navParamCur[1];
				pcdt.posZ = navParamCur[2];
				
				pcdt.veloX = navParamCur[3];
				pcdt.veloY = navParamCur[4];
				pcdt.veloZ = navParamCur[5];
				
				/*put to queue*/
				xQueueSend(INSToFlightConQueue,&pcdt,0);
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
						for(j=0;j<INS_FRAME_LEN;j++)
							insBufferCur[j] = IMU_delay_buffer[(i+buffer_header-GPS_DELAY_CNT)*INS_FRAME_LEN+j];
					}
					else
					{
						for(j=0;j<INS_FRAME_LEN;j++)
							insBufferCur[j] = IMU_delay_buffer[(i+buffer_header)*INS_FRAME_LEN+j];
					}
					insBufferCur[0] -= navParamK[6];
					insBufferCur[1] -= navParamK[7];
					insBufferCur[2] -= navParamK[8];
					
					INS_Update(navParamCur,insBufferCur);
				}
				Blinks(LED1,1);
			}
//			printf("%.1f %.1f %.1f %.1f %.1f\r\n",navParamK[0],navParamK[1],navParamK[2],navParamK[3],navParamK[4]);
		}
		else
		{
			/*data record*/
			sprintf(logData,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
											insBufferK[0],insBufferK[1],insBufferK[2],
											insBufferK[3],insBufferK[4],insBufferK[5],
											insBufferK[6],insBufferK[7],
											0.0,0.0,0.0,0.0,0.0,
											navParamK[3],navParamK[4]);
			xQueueSend(xDiskLogQueue,logData,0);
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

void INS_Update(float *navParam, float *IMU_data)
{
	float Cbn[9]={0.0};
	float dt = IMU_data[INDEX_DT];
	
	navParam[0] += navParam[3]*dt;
	navParam[1] += navParam[4]*dt;
	navParam[2] -= navParam[5]*dt;

	Quat2dcm(Cbn, IMU_data+3);
	navParam[3] += (Cbn[0]*IMU_data[0] + Cbn[1]*IMU_data[1] + Cbn[2]*IMU_data[2])*dt;
	navParam[4] += (Cbn[3]*IMU_data[0] + Cbn[4]*IMU_data[1] + Cbn[5]*IMU_data[2])*dt;
	navParam[5] += (Cbn[6]*IMU_data[0] + Cbn[7]*IMU_data[1] + Cbn[8]*IMU_data[2] + GRAVITY)*dt;
}

