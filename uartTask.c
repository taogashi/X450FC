#include "uartTask.h"
#include "UART.h"
#include "flightConTask.h"
#include <string.h>
#include <stdio.h>

xQueueHandle xUartGPSQueue;
xQueueHandle xUartVisionQueue;
xQueueHandle xUartWayPointQueue;

xQueueHandle xUartParaQueue;

//收到一帧数据后，将标志位置1
//读取后将标志位清零
u8 uart2Flag=0;
u8 uart3Flag=0;

char uart2Cache1[100];
char uart2Cache2[100];

char uart3Cache1[200];
char uart3Cache2[200];

#if USE_USART1
void USART1_IRQHandler(void)
{
	static u8 byteCNT=0;
	char byteRec;
//	u8 i;
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)!=RESET) 
	{
		USART_ClearFlag(USART1, USART_FLAG_ORE);
		USART_ReceiveData(USART1);
		byteCNT=0;
	} 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
		byteRec=USART_ReceiveData(USART1);
		if(byteRec != '\r' && byteRec!='\n' && byteRec!='\0')
		{	
			uart2Cache1[byteCNT++]=byteRec;
			if(byteCNT>=100) byteCNT=0;
		}
		else
		{	
			if(byteCNT > 1)
			{
				uart2Cache1[byteCNT]='\0';
				memcpy(uart2Cache2,uart2Cache1,byteCNT+1);
				uart2Flag=1;
			}
			byteCNT = 0;
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
#else
void USART2_IRQHandler(void)
{
	static u8 byteCNT=0;
	char byteRec;
//	u8 i;
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)!=RESET) 
	{
		USART_ClearFlag(USART2, USART_FLAG_ORE);
		USART_ReceiveData(USART2);
		byteCNT=0;
	} 
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{
		byteRec=USART_ReceiveData(USART2);
		if(byteRec != '\r' && byteRec!='\n')
		{	
			uart2Cache1[byteCNT++]=byteRec;
			if(byteCNT>=100) byteCNT=0;
		}
		else
		{	
			uart2Cache1[byteCNT]='\0';
			memcpy(uart2Cache2,uart2Cache1,byteCNT+1);
			uart2Flag=1;
			byteCNT = 0;
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}
#endif

void USART3_IRQHandler(void)
{
	static u8 byteCNT=0;
	char byteRec;
	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)!=RESET) 
	{
		USART_ClearFlag(USART3, USART_FLAG_ORE);
		USART_ReceiveData(USART3);
		byteCNT=0;
	} 
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{
		byteRec=USART_ReceiveData(USART3);
		if(byteRec=='$')
		{
			byteCNT=0;
			uart3Cache1[byteCNT++]=byteRec;
		}
		else if(byteRec != '\r' && byteRec!='\n')
		{	
			uart3Cache1[byteCNT++]=byteRec;
			if(byteCNT>=100) byteCNT=0;
		}
		else if(byteCNT != 0)
		{	
			uart3Cache1[byteCNT]='\0';
			memcpy(uart3Cache2,uart3Cache1,byteCNT+1);
			uart3Flag=1;
			byteCNT = 0;
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
}

void vUartRecTask(void* pvParameters)
{
	char buffer[100];
	char *keyword;
	u16 string_len;
	u16 index;
	u16 dummy;
	
	GGATypeDef ggaData;
	RMCTypeDef rmcData;
	GMVTypeDef gmvData;

	GPSDataType gpsData;
	VisionDataType vdt;
	
	WayPointType wpt;

	portTickType lastTime;
	portTickType curTime;
	
	USART3_Config();
	USART3_IT_Config();
	
	for(;;)
	{
		if(uart2Flag==1)
		{
			memcpy(buffer,uart2Cache2,100);
			uart2Flag=0;
			/******************* checking pos data **********************/
			if((keyword=strstr(buffer, "POS")) != NULL)
			{
				sscanf(keyword,"POS,%hd,%hd,%hd,%hd"
						,&(vdt.pos_x),&(vdt.pos_y),&(vdt.pos_z),&(vdt.chksm));
				if(vdt.pos_x + vdt.pos_y + vdt.pos_z == vdt.chksm)
				{
					xQueueSend(xUartVisionQueue, &vdt, 0);
				}
			}
			/********************  checking PID *************************/
			else if((keyword=strstr(buffer,"PID")) != NULL)
			{
				sscanf(keyword, "PID,%hu", &index);
				sscanf(keyword,PID_FORMAT_IN
						,&dummy
						,&(optional_param_global.loop_pid[index].xPID[0]), &(optional_param_global.loop_pid[index].xPID[1]), &(optional_param_global.loop_pid[index].xPID[2])
						,&(optional_param_global.loop_pid[index].yPID[0]), &(optional_param_global.loop_pid[index].yPID[1]), &(optional_param_global.loop_pid[index].yPID[2])
						,&(optional_param_global.loop_pid[index].zPID[0]), &(optional_param_global.loop_pid[index].zPID[1]), &(optional_param_global.loop_pid[index].zPID[2]));
				string_len = sprintf(buffer,PID_FORMAT_OUT
					,index, optional_param_global.loop_pid[index].xPID[0], optional_param_global.loop_pid[index].xPID[1], optional_param_global.loop_pid[index].xPID[2]
					,optional_param_global.loop_pid[index].yPID[0], optional_param_global.loop_pid[index].yPID[1], optional_param_global.loop_pid[index].yPID[2]
					,optional_param_global.loop_pid[index].zPID[0], optional_param_global.loop_pid[index].zPID[1], optional_param_global.loop_pid[index].zPID[2]);
				UartSend(buffer, string_len);
				curTime = lastTime = xTaskGetTickCount();
				while(uart2Flag==0 && curTime-lastTime<20000)
				{
					curTime= xTaskGetTickCount();
					vTaskDelay((portTickType)(10/portTICK_RATE_MS));
				}
				if(uart2Flag == 1)
				{
					memcpy(buffer,uart2Cache2,10);
					uart2Flag=0;
					if((keyword=strstr(buffer,"OK")) != NULL)
					{

						string_len = sprintf(buffer,"OK!\r\n");
						UartSend(buffer, string_len);
						
						xQueueSend(xUartParaQueue,&index, portMAX_DELAY);
						string_len = sprintf(buffer, "loaded!\r\n");
						UartSend(buffer, string_len);
					}
					else
					{
						string_len = sprintf(buffer,"abort!\r\n");
						UartSend(buffer, string_len);
					}
				}
				else
				{
					string_len = sprintf(buffer,"abort!\r\n");
					UartSend(buffer, string_len);
				}
			}
			/********************  checking Miscel param *************************/
			else if((keyword=strstr(buffer,"Miscel")) != NULL)
			{
				float temp_miscel[10];
				index = 4;
				sscanf(keyword,MISCEL_FORMAT_IN
						,&(temp_miscel[0]),&(temp_miscel[1])
						,&(temp_miscel[2]),&(temp_miscel[3])
						,&(temp_miscel[4]),&(temp_miscel[5])
						,&(temp_miscel[6]),&(temp_miscel[7])
						,&(temp_miscel[8]),&(temp_miscel[9]));
				string_len = sprintf(buffer, MISCEL_FORMAT_OUT
						,temp_miscel[0],temp_miscel[1]
						,temp_miscel[2],temp_miscel[3]
						,temp_miscel[4],temp_miscel[5]
						,temp_miscel[6],temp_miscel[7]
						,temp_miscel[8],temp_miscel[9]);
				UartSend(buffer,string_len);
				curTime = lastTime = xTaskGetTickCount();
				while(uart2Flag==0 && curTime-lastTime<20000)
				{
					curTime= xTaskGetTickCount();
					vTaskDelay((portTickType)(10/portTICK_RATE_MS));
				}
				if(uart2Flag == 1)
				{
					memcpy(buffer,uart2Cache2,10);
					uart2Flag=0;
					if((keyword=strstr(buffer,"OK")) != NULL)
					{
						memcpy(optional_param_global.miscel, temp_miscel, 10*sizeof(float));
						string_len = sprintf(buffer, "OK!\r\n");
						UartSend(buffer, string_len);
						xQueueSend(xUartParaQueue, &index, portMAX_DELAY);
						string_len = sprintf(buffer, "loaded!\r\n");
						UartSend(buffer, string_len);
					}
					else
					{
						string_len = sprintf(buffer, "abort!\r\n");
						UartSend(buffer, string_len);
					}
				}
				else
				{
					string_len = sprintf(buffer, "abort!\r\n");
					UartSend(buffer, string_len);
				}
			}
			
			/********************  checking Neutral param *************************/
			else if((keyword=strstr(buffer,"Neutral")) != NULL)
			{
				s16 temp_neutral[4];
				index = 5;
				sscanf(keyword,NEUTRAL_FORMAT_IN
						,&(temp_neutral[0]),&(temp_neutral[1])
						,&(temp_neutral[2]),&(temp_neutral[3]));
				string_len = sprintf(buffer, NEUTRAL_FORMAT_OUT
						,(temp_neutral[0]),(temp_neutral[1])
						,(temp_neutral[2]),(temp_neutral[3]));
				UartSend(buffer,string_len);
				curTime = lastTime = xTaskGetTickCount();
				while(uart2Flag==0 && curTime-lastTime<20000)
				{
					curTime= xTaskGetTickCount();
					vTaskDelay((portTickType)(10/portTICK_RATE_MS));
				}
				if(uart2Flag == 1)
				{
					memcpy(buffer,uart2Cache2,10);
					uart2Flag=0;
					if((keyword=strstr(buffer,"OK")) != NULL)
					{
						memcpy(optional_param_global.RCneutral, temp_neutral, 4*sizeof(s16));
						string_len = sprintf(buffer, "OK!\r\n");
						UartSend(buffer, string_len);
						xQueueSend(xUartParaQueue, &index, portMAX_DELAY);
						string_len = sprintf(buffer, "loaded!\r\n");
						UartSend(buffer, string_len);
					}
					else
					{
						string_len = sprintf(buffer, "abort!\r\n");
						UartSend(buffer, string_len);
					}
				}
				else
				{
					string_len = sprintf(buffer, "abort!\r\n");
					UartSend(buffer, string_len);
				}
			}
		}
		
		if(uart3Flag==1)
		{
			memcpy(buffer,uart3Cache2,100);
			uart3Flag=0;
			if((keyword=strstr(buffer,"GPGGA")) != NULL)
			{
				u16 Checksum=0;
				u8 N=strlen(keyword);
				u8 i;
				u8 CK_A=0;
				sscanf(keyword,"%*[^*]*%hx",&Checksum);
				for(i=0;i<N-3;i++)
				{
					CK_A^=keyword[i];
				}
				if(CK_A==Checksum)
				{
					sscanf(keyword,"GPGGA,%[^,],%lf,%c,%lf,%c,%c,%hd,%f,%f,%c,%[^,]"
							,ggaData.utc_time,&(ggaData.latitude_value),&(ggaData.latitude)
							,&(ggaData.longitude_value),&(ggaData.longitude),&(ggaData.position_fix_indicator)
							,&(ggaData.satellites_used),&(ggaData.HDOP),&(ggaData.MSL_altitude),&(ggaData.MSL_unit)
							,ggaData.other_info);
					gpsData.Lati=ggaData.latitude_value;
					gpsData.Long=ggaData.longitude_value;
					gpsData.Alti=ggaData.MSL_altitude;
					gpsData.type=GPGGA;
					if('A'==rmcData.status)
						xQueueSend(xUartGPSQueue,&gpsData,0);
				}
			}
			else if((keyword=strstr(buffer,"GPRMC")) != NULL)
			{
				u16 Checksum=0;
				u8 N=strlen(keyword);
				u8 i;
				u8 CK_A=0;
				sscanf(keyword,"%*[^*]*%hx",&Checksum);
				for(i=0;i<N-3;i++)
				{
					CK_A^=keyword[i];
				}
				if(CK_A==Checksum)
				{
					sscanf(keyword,"GPRMC,%[^,],%c,%lf,%c,%lf,%c,%f,%f,%[^,]"
							,rmcData.utc_time,&(rmcData.status)
							,&(rmcData.latitude_value),&(rmcData.latitude)
							,&(rmcData.longitude_value),&(rmcData.longitude)
							,&(rmcData.speed),&(rmcData.azimuth_angle)
							,rmcData.utc_data);
					gpsData.Lati=rmcData.latitude_value;
					gpsData.Long=rmcData.longitude_value;
					gpsData.SPD=rmcData.speed;
					gpsData.COG=rmcData.azimuth_angle;
					gpsData.type=GPRMC;
					if('A'==rmcData.status)
						xQueueSend(xUartGPSQueue,&gpsData,0);
				}
			}
			else if((keyword=strstr(buffer,"GPGMV")) != NULL)
			{
				u16 Checksum=0;
				u8 N=strlen(keyword);
				u8 i;
				u8 CK_A=0;
				sscanf(keyword,"%*[^*]*%hx",&Checksum);
				for(i=0;i<N-3;i++)
				{
					CK_A^=keyword[i];
				}
				if(CK_A==Checksum)
				{
					sscanf(keyword,"GPGMV,%[^,],%lf,%c,%lf,%c,%f,%f,%f,%f,%f"
							,gmvData.utc_time,&(gmvData.latitude_value)
							,&(gmvData.latitude),&(gmvData.longitude_value)
							,&(gmvData.longitude),&(gmvData.alti)
							,&(gmvData.speedE),&(gmvData.speedN)
							,&(gmvData.speedU),&(gmvData.azimuth_angle));
					gpsData.Lati=gmvData.latitude_value;
					gpsData.Long=gmvData.longitude_value;
					gpsData.Alti=gmvData.alti;
					gpsData.speedE = gmvData.speedE;
					gpsData.speedN = gmvData.speedN;
					gpsData.speedU = gmvData.speedU;
					gpsData.COG=gmvData.azimuth_angle;
					gpsData.type=GPGMV;

					if(gpsData.Lati > 100.0)
						xQueueSend(xUartGPSQueue,&gpsData,0);
				}
			}
			/********************  checking Waypoint *************************/
			else if((keyword=strstr(buffer,"Waypoint")) != NULL)
			{
				sscanf(keyword,WAYPOINT_FORMAT_IN
						,&(wpt.properties),&(wpt.maxSpeed),&(wpt.time),&(wpt.posAcrcy)
						,&(wpt.x),&(wpt.y),&(wpt.height),&(wpt.yaw));
				sprintf(buffer, WAYPOINT_FORMAT_OUT
						,(wpt.properties),(wpt.maxSpeed),(wpt.time),(wpt.posAcrcy)
						,(wpt.x),(wpt.y),(wpt.height),(wpt.yaw));
				UartSend(buffer, strlen(buffer));
				if(wpt.properties == 1)
				{
					xQueueSendToFront(xUartWayPointQueue,&wpt,0);
				}
				else if(wpt.properties == 0)
				{
					xQueueSend(xUartWayPointQueue,&wpt,0);
				}
			}
		}
//		sprintf(buffer, "hello world\r\n");
//		UartSend(buffer, strlen(buffer));
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}
}

