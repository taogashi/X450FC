#include "buttonTask.h"
#include "button.h"
#include "UART.h"
#include "ledTask.h"
#include "uartTask.h"
#include <stdio.h>
#include <string.h>

xQueueHandle xButtonQueueISR;
SystemRunMode SYSMODE = MODE1;

void EXTI3_IRQHandler(void)
{
	u8 data=1;
	static portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE;
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		xQueueSendFromISR(xButtonQueueISR,&data,&xHigherPriorityTaskWoken); 
  		if(xHigherPriorityTaskWoken == pdTRUE)
		{
		}
		/* Clear the EXTI line 15 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}


void vButtonEXTIHandler(void* vParameter)
{
	char printf_buffer[100];
	portBASE_TYPE xstatus;
//	SystemRunMode tempMode;
	portTickType sysTime,lastEXTItime;
	u8 RecData;
	u8 state=0;
	lastEXTItime = xTaskGetTickCount();
	Button_Config();
	Button_IT_Config();
	for(;;)
	{
		switch(state)
		{
			case 0:			//等待第一次按键
				xQueueReceive(xButtonQueueISR,&RecData,portMAX_DELAY);//等待第一个按键
				sysTime = xTaskGetTickCount();
				if(sysTime-lastEXTItime>20)
				{
					state=1;	
				}
				lastEXTItime = sysTime;
				break;
			case 1:
				xstatus=xQueueReceive(xButtonQueueISR,&RecData,(portTickType)600/portTICK_RATE_MS);//等待第二个按键
				sysTime = xTaskGetTickCount();		
				if(xstatus == pdPASS)
				{
					if(sysTime-lastEXTItime>20)
					{
						state=2;
					}
				}
				else
				{
					SYSMODE = MODE1;
					Blinks(LED1,1);
					sprintf(printf_buffer, "set to MODE%d\r\n",SYSMODE+1);
					UartSend(printf_buffer, strlen(printf_buffer));
					state = 0;					
				}
				lastEXTItime = sysTime;	
				break;
			case 2:
				xstatus=xQueueReceive(xButtonQueueISR,&RecData,(portTickType)600/portTICK_RATE_MS);//等待第三个按键
				sysTime = xTaskGetTickCount();		
				if(xstatus == pdPASS)
				{
					if(sysTime-lastEXTItime>20)
					{
						SYSMODE = MODE3;
						Blinks(LED1,3);
						sprintf(printf_buffer, "set to MODE%d\r\n",SYSMODE+1);
						UartSend(printf_buffer, strlen(printf_buffer));
					}
				}
				else
				{
					SYSMODE = MODE2;
					Blinks(LED1,2);
					sprintf(printf_buffer,"set to MODE%d\r\n",SYSMODE+1);	
					UartSend(printf_buffer, strlen(printf_buffer));
				}
				state = 0;
				lastEXTItime = sysTime;	
				break;
			default:
				state = 0;
				break;				
		}
	}
}
