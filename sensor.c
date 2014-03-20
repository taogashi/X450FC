#include "sensor.h"
#include "UART.h"
#include "buttonTask.h"
#include "pwm.h"

xQueueHandle xSenToAhrsQueue;

SensorDataType sdt;
SensorDataType localSDT={{0,0,0},{0,0,-9.8015},{0},0.0};

void vSenAHRSRead(void* pvParameters)
{
	portBASE_TYPE xstatus;
	u8 i;

	AHRS_SPI_Config();

	while(!ReadAHRSRaw(&localSDT))
	{
		vTaskDelay((portTickType)(100/portTICK_RATE_MS));
	}

	for(;;)
	{
		//ahrs data capture in 250Hz(4ms one period)
		xstatus = xQueueSend(xSenToAhrsQueue,&localSDT,(portTickType)(2/portTICK_RATE_MS));
		//队列满，采集新数据并加权滤波
		while(xstatus != pdPASS)
		{
//			RTOS_ENTER_CRITICAL();
			ReadAHRSRaw(&sdt);
//			RTOS_EXIT_CRITICAL();

			for(i=0;i<3;i++)
			{
				localSDT.gyr[i]=0.4*localSDT.gyr[i]+0.6*sdt.gyr[i];
				localSDT.acc[i]=0.4*localSDT.acc[i]+0.6*sdt.acc[i];
				localSDT.mag[i]=0.4*localSDT.mag[i]+0.6*sdt.mag[i];
			}
			for(i=0; i<4; i++)
			{
				localSDT.quaternion[i] = sdt.quaternion[i];
			}
			xstatus = xQueueSend(xSenToAhrsQueue,&localSDT,(portTickType)(2/portTICK_RATE_MS));
		}
		//程序设计10ms循环一次
//		RTOS_ENTER_CRITICAL();
		ReadAHRSRaw(&localSDT);
//		RTOS_EXIT_CRITICAL();
	}
}
