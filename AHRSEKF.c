#include "AHRSEKF.h"
#include "stm32f4xx.h"
#include <arm_math.h>
#include "flightConTask.h"
#include "kalman.h"
#include "axisTrans.h"
#include "sensor.h"

xQueueHandle AHRSToFlightConQueue;
xQueueHandle AHRSToINSQueue;


float insBuffer[INS_FRAME_LEN]={0};	//gyr,acc,mag,q,dt
									//dt comes last

GFilterType sensorGFT[6]={
{{0},10,9},
{{0},10,9},
{{0},10,9},
{{0},10,9},
{{0},10,9},
{{0},10,9}
};//acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]

float m0[3]={0.0};
float bodyQuat[4]={1,0,0,0};  //姿态四元数

const float P[16]={1,	0,	0, 0,
					0,	1,	0, 0,
					0,	0,	1, 0,
					0,  0,  0, 1};

#ifdef MEASURE_DIM3
const float R[9] = {
				100,0,0,
				0,100,0,
				0,0,100};
#endif

#ifdef MEASURE_DIM6
const float R[36]={
		1,  0,  0,	0,	0,	0, 
		0,  1,  0,	0,	0,	0,
		0,  0,  1,	0,	0,	0,
		0,	0,	0,	15,	0,	0,
		0,	0,	0,	0,	15,	0,
		0,	0,	0,	0,	0,	15};   //观测噪声协方差阵
#endif

const float Q[16]={
			0.00001,	0,			0,  	0,
			0,			0.00001,	0,	 	0,
			0,			0,			0.00001,0,
			0,    		0,      	0,  	0.00001};

//噪声协方差阵自适应
void SetR(SensorDataType *sdt,float *R,u8 measure_dim)
{	
	float totalAcc=sqrt(sdt->acc[0]*sdt->acc[0]+sdt->acc[1]*sdt->acc[1]+sdt->acc[2]*sdt->acc[2]);
	float gErr=fabs(totalAcc-9.814);
	u8 i;
	if(gErr<0.4)
	{
		for(i=0;i<3;i++)
			R[i*(measure_dim+1)]=0.2*(1+4/0.4*gErr)+0.8*R[i*(measure_dim+1)];			
	}
	else
	{
		for(i=0;i<3;i++)		
			R[i*(measure_dim+1)]=0.2*(5+15/9.814*(gErr-0.4))+0.8*R[i*(measure_dim+1)];
	}
}

/*------------------------------tasks----------------------------------------*/

void vAEKFAligTask(void* pvParameters)
{
	SensorDataType* pSDT;
	SensorDataType sdtBuffer;
	
	portTickType lastReadTime;
	float mAngle[3];
	float Cbn[9];
	u8 i=0,k;
	
	xQueueReceive(xSenToAhrsQueue,&pSDT,portMAX_DELAY);

	lastReadTime = xTaskGetTickCount();

	for(;;)
	{
		xQueueReceive(xSenToAhrsQueue,&pSDT,portMAX_DELAY);
		sdtBuffer = *pSDT;

		//数字滤波
		for(k=0;k<3;k++)
		{
			sdtBuffer.acc[k]=GaussianFilter(&(sensorGFT[k]),sdtBuffer.acc[k]);
			sdtBuffer.mag[k]=(s16)(GaussianFilter(&(sensorGFT[k+3]),(float)(sdtBuffer.mag[k])));
		}
	
		i++;
		vTaskDelayUntil(&lastReadTime,(portTickType)(40/portTICK_RATE_MS));
		if(i>50)
		{
			//calculate normalized geomagnetic
			float mag_norm;
			MeasureAngle(sdtBuffer.acc,sdtBuffer.mag,mAngle,mAngle,0);
			Angle2Quat(bodyQuat,mAngle);
			Quat2dcm(Cbn,bodyQuat);
			
			m0[0]=Cbn[0]*sdtBuffer.mag[0]+Cbn[1]*sdtBuffer.mag[1]+Cbn[2]*sdtBuffer.mag[2];
			m0[1]=Cbn[3]*sdtBuffer.mag[0]+Cbn[4]*sdtBuffer.mag[1]+Cbn[5]*sdtBuffer.mag[2];
			m0[2]=Cbn[6]*sdtBuffer.mag[0]+Cbn[7]*sdtBuffer.mag[1]+Cbn[8]*sdtBuffer.mag[2];
			arm_sqrt_f32(m0[0]*m0[0]+m0[1]*m0[1],&(m0[0]));
			m0[1]=0.0;		
			arm_sqrt_f32(m0[0]*m0[0]+m0[2]*m0[2],&mag_norm);
			m0[0] /= mag_norm;
			m0[2] /= mag_norm;
			
			xTaskCreate(vAEKFProcessTask, (signed portCHAR *)"AHRS_EKF", configMINIMAL_STACK_SIZE+1024, (void *)NULL, tskIDLE_PRIORITY+3,NULL);	
			vTaskDelete(NULL);
		}
	}
}

void vAEKFProcessTask(void* pvParameters)
{
	u8 i=0;	
	u8 k;

	float dt=0.005;
	ekf_filter filter;

#ifdef MEASURE_DIM3
	float measure[3]={0};
#endif

#ifdef MEASURE_DIM6
	float measure[6]={0};
	float Cbn[9]={0.0};
#endif

	float angle[3]={0};

	portTickType lastTime;

	SensorDataType* pSDT;
	SensorDataType sdtBuffer;

	float *p_insBuffer=insBuffer;
	AttConDataType acdt;
	
	filter=ekf_filter_new(4,6,Q,R,AHRS_GetA,AHRS_GetH,AHRS_aFunc,AHRS_hFunc);
	memcpy(filter->x,bodyQuat,filter->state_dim*sizeof(float));
	memcpy(filter->P,P,filter->state_dim*filter->state_dim*sizeof(float));
	xQueueReceive(xSenToAhrsQueue,&pSDT,portMAX_DELAY);

	lastTime = xTaskGetTickCount();
	for(;;)
	{
		xQueueReceive(xSenToAhrsQueue,&pSDT,portMAX_DELAY);
		sdtBuffer=*pSDT;

		acdt.rollAngleRate = sdtBuffer.gyr[0];
		acdt.pitchAngleRate = sdtBuffer.gyr[1];
		acdt.yawAngleRate = sdtBuffer.gyr[2];

		insBuffer[0]=sdtBuffer.acc[0];
		insBuffer[1]=sdtBuffer.acc[1];
		insBuffer[2]=sdtBuffer.acc[2];
		insBuffer[3]=filter->x[0];
		insBuffer[4]=filter->x[1];
		insBuffer[5]=filter->x[2];
		insBuffer[6]=filter->x[3];
		insBuffer[INDEX_DT]+=dt;
		
		//数字滤波
		for(k=0;k<3;k++)
		{
			sdtBuffer.acc[k]=GaussianFilter(&(sensorGFT[k]),sdtBuffer.acc[k]);
			sdtBuffer.mag[k]=(s16)(GaussianFilter(&(sensorGFT[k+3]),(float)(sdtBuffer.mag[k])));
		}
			
		EKF_predict(filter
					,(void *)(sdtBuffer.gyr)
					,(void *)NULL
					,(void *)(&dt)
					,(void *)(filter->A)
					,(void *)NULL);
					
		if(i++>=10)
		{
			float norm;
			i=0;			

			//获取观测量
			measure[0]=sdtBuffer.acc[0];
			measure[1]=sdtBuffer.acc[1];
			measure[2]=sdtBuffer.acc[2];
#ifdef MEASURE_DIM6
			measure[3]=sdtBuffer.mag[0];
			measure[4]=sdtBuffer.mag[1];
			measure[5]=sdtBuffer.mag[2];
#endif
			//normalize
			arm_sqrt_f32(measure[0]*measure[0]+measure[1]*measure[1]+measure[2]*measure[2],&norm);
			measure[0] /= norm;
			measure[1] /= norm;
			measure[2] /= norm;
#ifdef MEASURE_DIM6
			arm_sqrt_f32(measure[3]*measure[3]+measure[4]*measure[4]+measure[5]*measure[5],&norm);
			measure[3] /= norm;
			measure[4] /= norm;
			measure[5] /= norm;
#endif

			SetR(&sdtBuffer,filter->R,filter->measure_dim);
			
			EKF_update(filter
					,measure
					,(void *)(filter->x)
					,(void *)m0
					,(void *)(filter->x)
					,(void *)m0);

#ifdef MEASURE_DIM6
			//calculate m0, method from paper
			Quat2dcm(Cbn,filter->x);
			m0[0] = Cbn[0]*measure[3]+Cbn[1]*measure[4]+Cbn[2]*measure[5];
			m0[1] = Cbn[3]*measure[3]+Cbn[4]*measure[4]+Cbn[5]*measure[5];
			m0[2] = Cbn[6]*measure[3]+Cbn[7]*measure[4]+Cbn[8]*measure[5];
			arm_sqrt_f32(m0[0]*m0[0]+m0[1]*m0[1],&(m0[0]));
			m0[1]=0.0;		
#endif			
		}		
		QuatNormalize(filter->x);
		Quat2Angle(angle,filter->x);
		
		//赋值
		acdt.rollAngle = angle[0];
		acdt.pitchAngle = angle[1];
		acdt.yawAngle = angle[2];

		//发送数据
		xQueueSend(AHRSToFlightConQueue,&acdt,0);
		xQueueSend(AHRSToINSQueue,&p_insBuffer,0);

		vTaskDelayUntil(&lastTime,(portTickType)(5/portTICK_RATE_MS));
	}
}

/*
 * para1 gyro rate
 * para2 null
 * para3 dt
 * */
void AHRS_GetA(float *A,void *para1,void *para2,void *para3)
{
	float *w=(float *)para1;
	float DT=*(float *)para3 * 0.5;

	A[0]=1.0;				A[1]=-w[0]*DT;	A[2]=-w[1]*DT;	A[3]=-w[2]*DT;
	A[4]=w[0]*DT;		A[5]=1.0;				A[6]=w[2]*DT;		A[7]=-w[1]*DT;
	A[8]=w[1]*DT;		A[9]=-w[2]*DT;	A[10]=1.0;			A[11]=w[0]*DT;
	A[12]=w[2]*DT;	A[13]=w[1]*DT;	A[14]=-w[0]*DT;	A[15]=1.0;
}

void AHRS_GetH(float *H,void *para1,void *para2)
{
	float *q=(float *)para1;

#ifdef MEASURE_DIM6	
	float *m0_=(float *)para2;
#endif

//	float g=-9.814;
// g=-1;
	float q2[4];

	q2[0]=2*q[0];
	q2[1]=2*q[1];
	q2[2]=2*q[2];
	q2[3]=2*q[3];

	H[0]=q2[2];		H[1]=-q2[3];	H[2]=q2[0];		H[3]=-q2[1];
	H[4]=-q2[1];	H[5]=-q2[0];  H[6]=-q2[3];  H[7]=-q2[2];
	H[8]=-q2[0];  H[9]=q2[1];   H[10]=q2[2];  H[11]=-q2[3];

#ifdef MEASURE_DIM6
	H[12]=m0_[0]*q2[0]-m0_[2]*q2[2];   H[13]=m0_[0]*q2[1]+m0_[2]*q2[3];   H[14]=-m0_[0]*q2[2]-m0_[2]*q2[0]; H[15]=-m0_[0]*q2[3]+m0_[2]*q2[1];
	H[16]=-m0_[0]*q2[3]+m0_[2]*q2[1];  H[17]=m0_[0]*q2[2]+m0_[2]*q2[0];   H[18]=m0_[0]*q2[1]+m0_[2]*q2[3];  H[19]=-m0_[0]*q2[0]+m0_[2]*q2[2];
	H[20]=m0_[0]*q2[2]+m0_[2]*q2[0];   H[21]=m0_[0]*q2[3]-m0_[2]*q2[1];   H[22]=m0_[0]*q2[0]-m0_[2]*q2[2];  H[23]= m0_[0]*q2[1]+m0_[2]*q2[3];
#endif
}

/*
 * para1 matrix A
 * para2 NULL
 * */
void AHRS_aFunc(float *q,void *para1,void *para2)
{
	float *A=(float *)para1;
	arm_matrix_instance_f32 newqMat,qMat,AMat;

	newqMat.numRows=4;
	newqMat.numCols=1;
	newqMat.pData=pvPortMalloc(4*sizeof(float));

	qMat.numRows=4;
	qMat.numCols=1;
	qMat.pData=q;

	AMat.numRows=4;
	AMat.numCols=4;
	AMat.pData=A;

	arm_mat_mult_f32(&AMat,&qMat,&newqMat);
	memcpy(q,newqMat.pData,4*sizeof(float));
	vPortFree(newqMat.pData);
}

void AHRS_hFunc(float *hx,void *para1,void *para2)
{
	float *q = (float *)para1;
	float *m0_ = (float *)para2;
	
	float g[3]={0.0, 0.0, -1};
	arm_matrix_instance_f32 CbnMat,CnbMat
		,gMat,m0Mat,hx1Mat,hx2Mat;

	CbnMat.numRows=3;
	CbnMat.numCols=3;
	CbnMat.pData=pvPortMalloc(9*sizeof(float));

	CnbMat.numRows=3;
	CnbMat.numCols=3;
	CnbMat.pData=pvPortMalloc(9*sizeof(float));

	Quat2dcm(CbnMat.pData,q);
	arm_mat_trans_f32(&CbnMat,&CnbMat);
//	hx[3]=CbnMat.pData[0];
//	hx[4]=CbnMat.pData[1];
//	hx[5]=CbnMat.pData[2];
	vPortFree(CbnMat.pData);

	hx1Mat.numRows=3;
	hx1Mat.numCols=1;
	hx1Mat.pData=pvPortMalloc(3*sizeof(float));

	gMat.numRows=3;
	gMat.numCols=1;
	gMat.pData=g;

	arm_mat_mult_f32(&CnbMat,&gMat,&hx1Mat);

	hx2Mat.numRows=3;
	hx2Mat.numCols=1;
	hx2Mat.pData=pvPortMalloc(3*sizeof(float));

	m0Mat.numRows=3;
	m0Mat.numCols=1;
	m0Mat.pData=m0_;

#ifdef MEASURE_DIM6
	arm_mat_mult_f32(&CnbMat,&m0Mat,&hx2Mat);
#endif

	vPortFree(CnbMat.pData);

	hx[0]=hx1Mat.pData[0];
	hx[1]=hx1Mat.pData[1];
	hx[2]=hx1Mat.pData[2];

#ifdef MEASURE_DIM6
	hx[3]=hx2Mat.pData[0];
	hx[4]=hx2Mat.pData[1];
	hx[5]=hx2Mat.pData[2];
#endif

	vPortFree(hx1Mat.pData);
	vPortFree(hx2Mat.pData);
}
