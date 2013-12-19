#include "sensor.h"

xQueueHandle xSenToAhrsQueue;

SensorDataType sdt;
SensorDataType localSDT={{0,0,0},{0,0,-9.814},{0}};

void vSenAHRSRead(void* pvParameters)
{
	SensorDataType* pSDT;
	float gyr_offset[3]={0.0};

	portBASE_TYPE xstatus;
	u8 i=0;

	ADIS16405_SPI_Init();
	
//	RTOS_ENTER_CRITICAL();
	while(!ADIS16405_Burst_Read(&localSDT) || i++<20)
	{
		vTaskDelay((portTickType)(10/portTICK_RATE_MS));
	}
//	RTOS_EXIT_CRITICAL();
	pSDT = &localSDT;
	vTaskDelay((portTickType)(3000/portTICK_RATE_MS));
	
	/*calculate gyro offset*/
	for(i=0; i<100; i++)
	{
//		RTOS_ENTER_CRITICAL();
		ADIS16405_Burst_Read(&localSDT);
//		RTOS_EXIT_CRITICAL();
		
		gyr_offset[0] += localSDT.gyr[0];
		gyr_offset[1] += localSDT.gyr[1];
		gyr_offset[2] += localSDT.gyr[2];
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}
	
	gyr_offset[0] *= 0.01;
	gyr_offset[1] *= 0.01;
	gyr_offset[2] *= 0.01;
	
	for(;;)
	{
		//ahrs data capture in 250Hz(4ms one period)
		xstatus = xQueueSend(xSenToAhrsQueue,&pSDT,(portTickType)(4/portTICK_RATE_MS));
		//队列满，采集新数据并加权滤波
		while(xstatus != pdPASS)
		{
//			RTOS_ENTER_CRITICAL();
			ADIS16405_Burst_Read(&sdt);
//			RTOS_EXIT_CRITICAL();
			
			sdt.gyr[0] -= gyr_offset[0];
			sdt.gyr[1] -= gyr_offset[1];
			sdt.gyr[2] -= gyr_offset[2];

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
//		RTOS_ENTER_CRITICAL();
		ADIS16405_Burst_Read(&localSDT);
//		RTOS_EXIT_CRITICAL();
		localSDT.gyr[0] -= gyr_offset[0];
		localSDT.gyr[1] -= gyr_offset[1];
		localSDT.gyr[2] -= gyr_offset[2];
		
		pSDT = &localSDT;
	}
}
