#include "AHRSEKF.h"
#include "stm32f4xx.h"
#include <arm_math.h>
#include "kalman.h"
#include "axisTrans.h"
#include "sensor.h"
#include "filter.h"
#include "diskTask.h"
#include <stdio.h>

xQueueHandle AHRSToFlightConQueue;
xQueueHandle AHRSToINSQueue;
xQueueHandle AHRS2HeightQueue;

const float P[49]={	0.000049,	0,			0,			0,			0,		0,		0,
					0,			0.000049,	0, 			0,			0,		0,		0,
					0,			0,			0.000049, 	0,			0,		0,		0,
					0,  		0,  		0, 			0.000049,	0,		0,		0,
					0,			0,			0,			0,			0.1,	0,		0,
					0,			0,			0,			0,			0,		0.1,	0,
					0,			0,			0,			0,			0,		0,		0.1};

const float R[36]={
	0.01,  	0,  	0,		0,		0,		0, 
	0,  	0.01,  	0,		0,		0,		0,
	0,  	0,  	0.01,	0,		0,		0,
	0,		0,		0,		0.04,	0,		0,
	0,		0,		0,		0,		0.04,	0,
	0,		0,		0,		0,		0,		0.04};   //观测噪声协方差阵

const float Q[49]={
			0.000004,	0,			0,  		0,			0,				0,				0,
			0,			0.000004,	0,			0,			0,				0,				0,
			0,			0,			0.000004,	0,			0,				0,				0,
			0,    		0,      	0,  		0.000004,	0,				0,				0,
			0,			0,			0,			0,			0.000000000001,	0,				0,
			0,			0,			0,			0,			0,				0.000000000001,	0,
			0,			0,			0,			0,			0,				0,				0.000000000001};

//噪声协方差阵自适应
void SetR(SensorDataType *sdt,float *R,u8 measure_dim)
{	
	float totalAcc=sqrt(sdt->acc[0]*sdt->acc[0]+sdt->acc[1]*sdt->acc[1]+sdt->acc[2]*sdt->acc[2]);
	float gErr=fabs(totalAcc-9.8015);
	u8 i;
	if(gErr<0.4)
	{
		for(i=0;i<3;i++)
			R[i*(measure_dim+1)]=0.2*(0.01+0.24/0.4*gErr)+0.8*R[i*(measure_dim+1)];			
	}
	else
	{
		for(i=0;i<3;i++)		
			R[i*(measure_dim+1)]=0.2*(0.25+0.75/9.8015*(gErr-0.4))+0.8*R[i*(measure_dim+1)];
	}
}

/*------------------------------tasks----------------------------------------*/
void vAEKFProcessTask(void* pvParameters)
{	
	char print_buffer[100];
	u16 string_len;
	
	/*index*/
	u8 i=0;	
	u8 k;

	/*sensor data*/
	SensorDataType sdt;

	/*geomagn info*/
	float m0[3]={0.0};
	float mag_norm;
	
	/*att representation*/
	float angle[3]={0};
	float bodyQuat[4]={1,0,0,0};
	float Cbn[9];

	/*FIR filter*/
	GFilterType sensorGFT[6]={
	{{0},10,9},
	{{0},10,9},
	{{0},10,9},
	{{0},10,9},
	{{0},10,9},
	{{0},10,9}
	};//acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]

	/*kalman filter*/
	float dt=0.005;
	ekf_filter filter;
	float measure[6]={0};
	
	/*result*/
	AHRSDataType acdt;
	AHRS2INSType a2it;
	
	portTickType lastTime;
	
	//initialize FIR filter
	while(i<50)
	{
		xQueueReceive(xSenToAhrsQueue, &sdt, portMAX_DELAY);
		for(k=0;k<3;k++)
		{
			sdt.acc[k]=GaussianFilter(&(sensorGFT[k]),sdt.acc[k]);
			sdt.mag[k]=(s16)(GaussianFilter(&(sensorGFT[k+3]),(float)(sdt.mag[k])));
		}
		i++;
		vTaskDelay((portTickType)20/portTICK_RATE_MS);
	}
	
	/*initialize attitude*/
	MeasureAngle(sdt.acc,sdt.mag,angle,angle,0);
	Angle2Quat(bodyQuat,angle);
	Quat2dcm(Cbn,bodyQuat);
	
	/*initialize geomagn info*/	
	m0[0] = Cbn[0]*sdt.mag[0] + Cbn[1]*sdt.mag[1] + Cbn[2]*sdt.mag[2];
	m0[1] = Cbn[3]*sdt.mag[0] + Cbn[4]*sdt.mag[1] + Cbn[5]*sdt.mag[2];
	m0[2] = Cbn[6]*sdt.mag[0] + Cbn[7]*sdt.mag[1] + Cbn[8]*sdt.mag[2];
	arm_sqrt_f32(m0[0]*m0[0]+m0[1]*m0[1],&(m0[0]));
	m0[1]=0.0;		
	arm_sqrt_f32(m0[0]*m0[0]+m0[2]*m0[2],&mag_norm);
	m0[0] /= mag_norm;
	m0[2] /= mag_norm;
	
	/*initialize kalman filter*/
	filter=ekf_filter_new(7,6,Q,R,AHRS_GetA,AHRS_GetH,AHRS_aFunc,AHRS_hFunc);
	filter->x[0] = bodyQuat[0];	filter->x[1] = bodyQuat[1];	filter->x[2] = bodyQuat[2];	filter->x[3] = bodyQuat[3];
	filter->x[4] = 1.0;	filter->x[5] = 1.0;	filter->x[6] = 1.0;
	memcpy(filter->P,P,filter->state_dim*filter->state_dim*sizeof(float));
	
	lastTime = xTaskGetTickCount();
	/*filter loop*/
	for(;;)
	{
		/*read sensor data*/
		xQueueReceive(xSenToAhrsQueue, &sdt, portMAX_DELAY);
		sdt.gyr[0] = 2*sdt.gyr[0];
		
		/*fill INS_frame_buffer with un-filtered data*/
		a2it.acc[0] = sdt.acc[0];
		a2it.acc[1] = sdt.acc[1];
		a2it.acc[2] = sdt.acc[2];
		a2it.q[0] = filter->x[0];
		a2it.q[1] = filter->x[1];
		a2it.q[2] = filter->x[2];
		a2it.q[3] = filter->x[3];
		a2it.height = sdt.height;
		a2it.dt += dt;
		
		/*smooth acc and magn data*/
		for(k=0;k<3;k++)
		{
			sdt.acc[k]=GaussianFilter(&(sensorGFT[k]),sdt.acc[k]);
			sdt.mag[k]=(s16)(GaussianFilter(&(sensorGFT[k+3]),(float)(sdt.mag[k])));
		}
			
		EKF_predict(filter
					,(void *)(filter->x)
					,(void *)(sdt.gyr)
					,(void *)(&dt)
					,(void *)NULL);
					
		if(i++>=20)
		{			
			float norm;
			i=0;
			
			/*get measurement*/
			measure[0]=sdt.acc[0];
			measure[1]=sdt.acc[1];
			measure[2]=sdt.acc[2];
			measure[3]=sdt.mag[0];
			measure[4]=sdt.mag[1];
			measure[5]=sdt.mag[2];

			//normalize
			arm_sqrt_f32(measure[0]*measure[0]+measure[1]*measure[1]+measure[2]*measure[2],&norm);
			measure[0] /= norm;
			measure[1] /= norm;
			measure[2] /= norm;

			arm_sqrt_f32(measure[3]*measure[3]+measure[4]*measure[4]+measure[5]*measure[5],&norm);
			measure[3] /= norm;
			measure[4] /= norm;
			measure[5] /= norm;

			/*matrix R auto adaptive*/
			SetR(&sdt,filter->R,filter->measure_dim);
			
			EKF_update(filter
					,measure
					,(void *)(filter->x)
					,(void *)m0
					,(void *)NULL
					,(void *)NULL);

			//calculate m0, method from paper
			Quat2dcm(Cbn,filter->x);
			m0[0] = Cbn[0]*measure[3]+Cbn[1]*measure[4]+Cbn[2]*measure[5];
			m0[1] = Cbn[3]*measure[3]+Cbn[4]*measure[4]+Cbn[5]*measure[5];
			m0[2] = Cbn[6]*measure[3]+Cbn[7]*measure[4]+Cbn[8]*measure[5];
			arm_sqrt_f32(m0[0]*m0[0]+m0[1]*m0[1],&(m0[0]));
			m0[1]=0.0;
			
			string_len = sprintf(print_buffer, "%.2f %.2f %.2f %.2f %.2f %.2f\r\n"
											, angle[0]*57.3, angle[1]*57.3, angle[2]*57.3
											, filter->x[4], filter->x[5], filter->x[6]);
			UartSend(print_buffer, string_len);
		}		
		QuatNormalize(filter->x);
		Quat2Angle(angle,filter->x);
		
		/*fill data to flight controll*/
		acdt.rollAngle = angle[0];
		acdt.pitchAngle = angle[1];
		acdt.yawAngle = angle[2];
		acdt.rollAngleRate = sdt.gyr[0];
		acdt.pitchAngleRate = sdt.gyr[1];
		acdt.yawAngleRate = sdt.gyr[2];
		
		/*push to queue*/
		xQueueSend(AHRSToFlightConQueue,&acdt,0);
		if(pdPASS == xQueueSend(AHRSToINSQueue,&a2it,0))
		{
			a2it.dt = 0.0;
		}
		xQueueSend(AHRS2HeightQueue,&a2it,0);

		vTaskDelayUntil(&lastTime,(portTickType)(5/portTICK_RATE_MS));
	}
}

/*
 * para1 gyro rate
 * para2 null
 * para3 dt
 * */
void AHRS_GetA(float *A,void *para1,void *para2,void *para3, void *para4)
{
	float *x = (float *)para1;
	float *w = (float *)para2;
	float DT = *(float *)para3 * 0.5;
	
	float rate_1xdt = w[0]*DT;
	float rate_2xdt = w[1]*DT;
	float rate_3xdt = w[2]*DT;
	
	float true_rate_1xdt = w[0]*x[4]*DT;
	float true_rate_2xdt = w[1]*x[5]*DT;
	float true_rate_3xdt = w[2]*x[6]*DT;

	memset(A, 0, 196);

	A[0]=1.0;				A[1]=-true_rate_1xdt;	A[2]=-true_rate_2xdt;	A[3]=-true_rate_3xdt; 	A[4]=-rate_1xdt*x[1]; 	A[5]=-rate_2xdt*x[2]; 	A[6]=-rate_3xdt*x[3];
	A[7]=true_rate_1xdt;	A[8]=1.0;				A[9]=true_rate_3xdt;	A[10]=-true_rate_2xdt;	A[11]=rate_1xdt*x[0];	A[12]=-rate_2xdt*x[3];	A[13]=rate_3xdt*x[2];
	A[14]=true_rate_2xdt;	A[15]=-true_rate_3xdt;	A[16]=1.0;				A[17]=true_rate_1xdt;	A[18]=rate_1xdt*x[3];	A[19]=rate_2xdt*x[0];	A[20]=-rate_3xdt*x[1];
	A[21]=true_rate_3xdt;	A[22]=true_rate_2xdt;	A[23]=-true_rate_1xdt;	A[24]=1.0;				A[25]=-rate_1xdt*x[2];	A[26]=rate_2xdt*x[1];	A[27]=rate_3xdt*x[0];				
	A[32] = 1.0;
	A[40] = 1.0;
	A[48] = 1.0;
}

/*
 * para1 matrix A
 * para2 NULL
 * */
void AHRS_aFunc(float *x,void *para1,void *para2,void *para3, void *para4)
{
	float *w = (float *)para2;
	float DT = *(float *)para3 * 0.5;
	
	float s1xw1xdt = x[4]*w[0]*DT;
	float s2xw2xdt = x[5]*w[1]*DT;
	float s3xw3xdt = x[6]*w[2]*DT;
	
	arm_matrix_instance_f32 newqMat,qMat,AMat;

	newqMat.numRows=4;
	newqMat.numCols=1;
	newqMat.pData=pvPortMalloc(4*sizeof(float));

	qMat.numRows=4;
	qMat.numCols=1;
	qMat.pData=x;

	AMat.numRows=4;
	AMat.numCols=4;
	AMat.pData=pvPortMalloc(16*sizeof(float));
	
	AMat.pData[0]=1.0;			AMat.pData[1]=-s1xw1xdt;	AMat.pData[2]=-s2xw2xdt;	AMat.pData[3] = -s3xw3xdt;
	AMat.pData[4]=s1xw1xdt;		AMat.pData[5]=1.0;			AMat.pData[6]=s3xw3xdt;		AMat.pData[7] = -s2xw2xdt;
	AMat.pData[8]=s2xw2xdt;		AMat.pData[9]=-s3xw3xdt;	AMat.pData[10]=1.0;			AMat.pData[11] = s1xw1xdt;
	AMat.pData[12]=s3xw3xdt;	AMat.pData[13]=s2xw2xdt;	AMat.pData[14]=-s1xw1xdt;	AMat.pData[15] = 1.0;

	arm_mat_mult_f32(&AMat,&qMat,&newqMat);
	memcpy(x,newqMat.pData,4*sizeof(float));
	vPortFree(newqMat.pData);
	vPortFree(AMat.pData);
}

void AHRS_GetH(float *H,void *para1,void *para2,void *para3, void *para4)
{
	float *q=(float *)para1;
	float *m0_=(float *)para2;

	float q2[4];
	
	memset(H, 0, 168);

	q2[0]=2*q[0];
	q2[1]=2*q[1];
	q2[2]=2*q[2];
	q2[3]=2*q[3];

	H[0]=q2[2];		H[1]=-q2[3];	H[2]=q2[0];		H[3]=-q2[1];	H[4]=0.0;	H[5]=0.0;	H[6]=0.0;
	H[7]=-q2[1];	H[8]=-q2[0];  	H[9]=-q2[3];  	H[10]=-q2[2];	H[11]=0.0;	H[12]=0.0;	H[13]=0.0;
	H[14]=-q2[0];  	H[15]=q2[1];   	H[16]=q2[2];  	H[17]=-q2[3];	H[18]=0.0;	H[19]=0.0;	H[20]=0.0;

	H[21]=m0_[0]*q2[0]-m0_[2]*q2[2];   H[22]=m0_[0]*q2[1]+m0_[2]*q2[3];   H[23]=-m0_[0]*q2[2]-m0_[2]*q2[0]; H[24]=-m0_[0]*q2[3]+m0_[2]*q2[1];	H[25]=0.0;	H[26]=0.0;	H[27]=0.0;
	H[28]=-m0_[0]*q2[3]+m0_[2]*q2[1];  H[29]=m0_[0]*q2[2]+m0_[2]*q2[0];   H[30]=m0_[0]*q2[1]+m0_[2]*q2[3];  H[31]=-m0_[0]*q2[0]+m0_[2]*q2[2];	H[32]=0.0;	H[33]=0.0;	H[34]=0.0;
	H[35]=m0_[0]*q2[2]+m0_[2]*q2[0];   H[36]=m0_[0]*q2[3]-m0_[2]*q2[1];   H[37]=m0_[0]*q2[0]-m0_[2]*q2[2];  H[38]= m0_[0]*q2[1]+m0_[2]*q2[3];	H[39]=0.0;	H[40]=0.0;	H[41]=0.0;
}

void AHRS_hFunc(float *hx,void *para1,void *para2,void *para3,void *para4)
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

	arm_mat_mult_f32(&CnbMat,&m0Mat,&hx2Mat);

	vPortFree(CnbMat.pData);

	hx[0]=hx1Mat.pData[0];
	hx[1]=hx1Mat.pData[1];
	hx[2]=hx1Mat.pData[2];

	hx[3]=hx2Mat.pData[0];
	hx[4]=hx2Mat.pData[1];
	hx[5]=hx2Mat.pData[2];

	vPortFree(hx1Mat.pData);
	vPortFree(hx2Mat.pData);
}
