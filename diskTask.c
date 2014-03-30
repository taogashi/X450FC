#include "diskTask.h"
#include <stdio.h>
#include "buttonTask.h"
#include "flightConTask.h"
#include "UART.h"
#include "sensor.h"

xQueueHandle xDiskParamQueue;
xQueueHandle xDiskLogQueue;

FRESULT scanFiles(char* path)
{
	FRESULT res;
	FILINFO finfo;
	DIR dir;
#if _USE_LFN
	static char lfn[_MAX_LFN * (_DF1S ? 2:1)+1];
	finfo.lfname=lfn;
	finfo.lfsize = sizeof(lfn);
#endif

	res=f_opendir(&dir,path);
	if(res == FR_OK)
	{
		while (f_readdir(&dir, &finfo) == FR_OK)  	 //循环依次读取文件名
		{	 
			if (finfo.fattrib & AM_ARC) 			     //判断文件属性是否为存档型	 TXT文件一般都为存档型
			{
				if(!finfo.fname[0])	 break;    		     //如果是文件名为空表示到目录的末尾。退出
				else 
				{	 
					printf("\r\n文件名是：%s\r\n",finfo.fname);	  //输出文件名
				}
			}
		}
	}
	return res;
}

void vDiskOperation(void* vParameter)
{
	char printf_buffer[100];
	u16 string_len;
	
	FATFS fs;            // Work area (file system object) for logical drive
	FIL paraFile,logFile;      //,  file objects
	FRESULT res;         // FatFs function common result code
//	UINT br, bw;         // File R/W count
//	DIR dirs;
//	char path[100]={""};  //磁盘根目录
//
	portBASE_TYPE xstatus;
	char data[256];
	unsigned int  ByteRead;
	unsigned int  ByteWrite;
	u8 logState=0;
	
	u16 index;
	s16 check;
	
	memset(&optional_param_global, 0, sizeof(OptionalPara));
	vTaskDelay((portTickType)(300/portTICK_RATE_MS));
	disk_initialize(0);
	res=f_mount(0, &fs);
	
	vTaskDelay((portTickType)(200/portTICK_RATE_MS));
	//open the parameter file
	res = f_open(&paraFile,"quad.par",FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
	if(res != FR_OK)
	{
		string_len = sprintf(printf_buffer, "failed to open file!\r\n");
		UartSend(printf_buffer, string_len);
	}
	else
	{
		res = f_lseek(&paraFile, 0);
		res = f_read(&paraFile,data,sizeof(OptionalPara),&ByteRead);
		res = f_lseek(&paraFile, 0);
		//valid parameter contains more than 10 bytes, no doubt
		if(ByteRead == sizeof(OptionalPara))
		{
			optional_param_global = *(OptionalPara *)data;
			check = CalChecksum(&optional_param_global);
			if(check == optional_param_global.checksum)
			{
				//put parameters to the queue
				xQueueSend(xDiskParamQueue,&index,0);
				//make sure the parameters is received by flightConTask
//				xstatus = xQueuePeek(xDiskParamQueue,&index,0);
//				while(xstatus == pdPASS)
//				{
//					vTaskDelay((portTickType)10/portTICK_RATE_MS);
//					xstatus = xQueuePeek(xDiskParamQueue,&index,0);
//				}
				vTaskDelay((portTickType)2000/portTICK_RATE_MS);
			}		
		}
	}
	res = f_close(&paraFile);
	for(;;)
	{
		//写入参数
		xstatus=xQueueReceive(xDiskParamQueue,&index,0);
		if(xstatus == pdTRUE)
		{
			*(OptionalPara *)data = optional_param_global;
			
			res = f_open(&paraFile,"quad.par",FA_WRITE | FA_OPEN_ALWAYS);
			if(res != FR_OK)
			{
				string_len = sprintf(printf_buffer, "failed to open file!\r\n");
				UartSend(printf_buffer, string_len);
			}
			res = f_lseek(&paraFile, 0);
			//写入悬停时的油门
			res = f_write(&paraFile,data,sizeof(OptionalPara),&ByteRead);
			res = f_close(&paraFile);
		}

		//记录数据
		
		switch(logState)
		{
			case 0:
				if(GetSystemMode()==MODE2)
				{
					f_unlink("log.txt");
					res = f_open(&logFile,"log.txt",FA_WRITE | FA_OPEN_ALWAYS);
					logState=1;
				}
				else
				{
//					printf("%.3f %.3f %d %lf %lf\r\n",sdt.gyr[0],sdt.acc[0],sdt.mag[0],gdt.Lati,gdt.Long);
					vTaskDelay((portTickType)(20/portTICK_RATE_MS));
				}
				break;
			case 1:
				if(GetSystemMode()==MODE1)
				{
					f_close(&logFile);
					logState=0;
				}
				else
				{
					xQueueReceive(xDiskLogQueue,data,portMAX_DELAY);
					f_write(&logFile,data,strlen(data),&ByteWrite);	
				}
				break;
			default:
				vTaskDelay((portTickType)(20/portTICK_RATE_MS));
				break;
		}
	}
}
