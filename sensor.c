#include "sensor.h"

xQueueHandle xSenToAhrsQueue;

SensorDataType sdt;
SensorDataType localSDT={{0,0,0},{0,0,-9.814},{0}};

const float GCoef[5][10]={{0.2146,0.2051,0.1792,0.1431,0.1045,0.0697,0.0425,0.0237,0.0120,0.0056},  //10
					{0.2357,0.2229,0.1887,0.1429,0.0969,0.0588,0.0319,0.0155,0.0067,0.0},
					{0.2613,0.2436,0.1973,0.1388,0.0848,0.0451,0.0208,0.0083,0.0   ,0.0},
					{0.2933,0.2675,0.2031,0.1283,0.0675,0.0295,0.0108,0.0   ,0.0   ,0.0},
					{0.3341,0.2949,0.2027,0.1085,0.0452,0.0147,0.0   ,0.0   ,0.0   ,0.0}};

float GaussianFilter(GFilterType *gft,float newData)
{
	u8 i;
	float output=0.0;
	gft->pool[gft->head]=newData;
	for(i=0;i<gft->length;i++)
	{
		output+=gft->pool[(gft->head+i)%(gft->length)]*GCoef[10-gft->length][i];
	}
	if(gft->head==0) gft->head=gft->length-1;
	else gft->head--;

	if(gft->pool[gft->head]<0.00001 && gft->pool[gft->head]>-0.00001)
		return newData;
	return output;
}

//去毛刺
float Debur(GFilterType *gft,float newData,float thre)
{
	u8 i;
	float av=0.0;
//	gft->pool[gft->head]=newData;
	for(i=0;i<gft->length;i++)
	{
		av+=gft->pool[(gft->head+i)%(gft->length)]*GCoef[10-gft->length][i];
	}
	if(newData-av>thre || newData-av<(0-thre)) newData=av;
	else gft->pool[gft->head]=newData;

	if(gft->head==0) gft->head=gft->length-1;
	else gft->head--;

	return newData;	
}

void GyrDebur(float *gyr)
{
	static float lastGyr[3]={0.0};
	static u8 errCNT[3]={0};
	u8 k;
	for(k=0;k<3;k++)
	{
		if(lastGyr[k]-gyr[k] > 0.35 || lastGyr[k]-gyr[k] < -0.35)
		{
			errCNT[k]++;
			if(errCNT[k]>=2)
			{ 
				lastGyr[k]=gyr[k];
				errCNT[k]=0;
			}
			else
			{
				gyr[k]=lastGyr[k];
			}
		} 
		else
		{
				lastGyr[k]=gyr[k];
				errCNT[k]=0;
		}
	}
}

void vSenAHRSRead(void* pvParameters)
{
	SensorDataType* pSDT;

	portBASE_TYPE xstatus;
	u8 i;

	ADIS16405_SPI_Init();
	
	RTOS_ENTER_CRITICAL();
	while(!ADIS16405_Burst_Read(&localSDT))
	{
		vTaskDelay((portTickType)(10/portTICK_RATE_MS));
	}
	RTOS_EXIT_CRITICAL();
	pSDT = &localSDT;
	vTaskDelay((portTickType)(1000/portTICK_RATE_MS));
	for(;;)
	{
		//ahrs data capture in 250Hz(4ms one period)
		xstatus = xQueueSend(xSenToAhrsQueue,&pSDT,(portTickType)(4/portTICK_RATE_MS));
		//队列满，采集新数据并加权滤波
		while(xstatus != pdPASS)
		{
			RTOS_ENTER_CRITICAL();
			ADIS16405_Burst_Read(&sdt);
			RTOS_EXIT_CRITICAL();

			for(i=0;i<3;i++)
			{				
				localSDT.gyr[i]=0.5*localSDT.gyr[i]+0.5*sdt.gyr[i];
				localSDT.acc[i]=0.5*localSDT.acc[i]+0.5*sdt.acc[i];
				localSDT.mag[i]=0.5*localSDT.mag[i]+0.5*sdt.mag[i];
			}
			pSDT = &localSDT;
			xstatus = xQueueSend(xSenToAhrsQueue,&pSDT,(portTickType)(4/portTICK_RATE_MS));
		}
		//程序设计10ms循环一次
		RTOS_ENTER_CRITICAL();
		ADIS16405_Burst_Read(&localSDT);
		RTOS_EXIT_CRITICAL();
		pSDT = &localSDT;
	}
}
