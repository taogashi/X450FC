#include "diskTask.h"
#include <stdio.h>
#include "buttonTask.h"
#include "flightConTask.h"
#include "UART.h"
#include "sensor.h"

xQueueHandle xDiskWrQueue1;
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
	
	FATFS fs;            // Work area (file system object) for logical drive
	FIL paraFile,logFile;      //,  file objects
	FRESULT res;         // FatFs function common result code
//	UINT br, bw;         // File R/W count
//	DIR dirs;
//	char path[100]={""};  //磁盘根目录
//
	portBASE_TYPE xstatus;
	char data[150];
	unsigned int  ByteRead;
	unsigned int  ByteWrite;
	u8 logState=0;

	OptionalPara OP={215.5,80.5,0.8 //rollPID
			,130.5,100.4,70.3,0.5 //yawPID
			,1.0,1.0,0.0		//horizontal pos PID
			,0.5,0.33,0.00		//vertica pos PID
			,0.48 				//hover thrust out
			,1525,1525,1112,1526 //RC neutral
			,0};  
	
	s16 check;

	disk_initialize(0);
	res=f_mount(0, &fs);

	OP.checksum = CalChecksum(&OP);
	
	//open the parameter file
	res = f_open(&paraFile,"para.txt",FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
	if(res != FR_OK)
	{
		sprintf(printf_buffer, "failed to open file!\r\n");
		UartSend(printf_buffer, strlen(printf_buffer));
	}
	else
	{
		res = f_lseek(&paraFile, 0);
		res = f_read(&paraFile,data,150,&ByteRead);
		res = f_lseek(&paraFile, 0);
		//valid parameter contains more than 10 bytes, no doubt
		if(ByteRead>10)
		{
			sscanf(data,PARA_FORMAT_IN
					,&(OP.rollP),&(OP.rollD),&(OP.rollI)
					,&(OP.yawP),&(OP.yawD1),&(OP.yawD2),&(OP.yawI)
					,&(OP.horiP),&(OP.horiD),&(OP.horiI)
					,&(OP.heightP),&(OP.heightD),&(OP.heightI)
					,&(OP.hoverThrust)
					,&(OP.RCneutral[0]),&(OP.RCneutral[1]),&(OP.RCneutral[2]),&(OP.RCneutral[3])
					,&(OP.checksum));
			check = CalChecksum(&OP);
			if(check == OP.checksum)
			{
				//put parameters to the queue
				xQueueSend(xDiskWrQueue1,&OP,0);
				//make sure the parameters is received by flightConTask
				xstatus = xQueuePeek(xDiskWrQueue1,&OP,0);
				while(xstatus == pdPASS)
				{
					vTaskDelay((portTickType)10/portTICK_RATE_MS);
					xstatus = xQueuePeek(xDiskWrQueue1,&OP,0);
				}
			}		
		}
	}
	res = f_close(&paraFile);
	for(;;)
	{
		//写入参数
		xstatus=xQueueReceive(xDiskWrQueue1,&OP,0);
		if(xstatus == pdTRUE)
		{
			sprintf(data,PARA_FORMAT_OUT
					,(OP.rollP),(OP.rollD),(OP.rollI)
					,(OP.yawP),(OP.yawD1),(OP.yawD2),(OP.yawI)
					,(OP.horiP),(OP.horiD),(OP.horiI)
					,(OP.heightP),(OP.heightD),(OP.heightI)
					,(OP.hoverThrust)
					,(OP.RCneutral[0]),(OP.RCneutral[1]),(OP.RCneutral[2]),(OP.RCneutral[3])
					,(OP.checksum));
			res = f_open(&paraFile,"para.txt",FA_WRITE | FA_OPEN_ALWAYS);
			if(res != FR_OK)
			{
				sprintf(printf_buffer, "failed to open file!\r\n");
				UartSend(printf_buffer, strlen(printf_buffer));
			}
			res = f_lseek(&paraFile, 0);
			//写入悬停时的油门
			res = f_write(&paraFile,data,strlen(data),&ByteRead);
			res = f_close(&paraFile);
		}

		//记录数据
		
		switch(logState)
		{
			case 0:
				if(SYSMODE==MODE2)
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
				if(SYSMODE==MODE1)
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
