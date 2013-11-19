#ifndef _UARTTASK_H_
#define _UARTTASK_H_

#include "OSConfig.h"
#include "stm32f4xx.h"

#define GPGGA 0
#define GPRMC 1
#define GPGMV 2

typedef struct
{
	char utc_time[11];
	double latitude_value;	 
	char latitude;
	double longitude_value;
	char longitude;
	char position_fix_indicator;
	u16 satellites_used;	   //??¦Ì?¦Ì??¨¤D?¨ºy
	float HDOP;
	float MSL_altitude;
	char MSL_unit;
	char other_info[20];
}GGATypeDef;	

typedef struct
{
	char utc_time[11]; 
	char status; 
	double latitude_value; 	//?3?¨¨
	char latitude; 			 //???3?1¨º?¡À¡À?3
	double longitude_value; 	 //?-?¨¨
	char longitude; 		 //???-?1¨º??¡Â?-
	float speed; 			 //¦Ì??¨´
	float azimuth_angle; 	 //¦Ì??¨´¡¤??¨°
	char utc_data[7]; 
}RMCTypeDef;	

typedef struct
{
	char utc_time[11];  
	double latitude_value; 	//?3?¨¨
	char latitude; 			 //???3?1¨º?¡À¡À?3
	double longitude_value; 	 //?-?¨¨
	char longitude; 		 //???-?1¨º??¡Â?-
	float alti;
	float speedE; 			 //¦Ì??¨´
	float speedN;
	float speedU;
	float azimuth_angle; 	 //¦Ì??¨´¡¤??¨°
}GMVTypeDef;

typedef struct
{
	double Lati;
	double Long;
	float Alti;
	float SPD;//speed over ground
	float COG;//course over ground
//in MXT GPGMV
	float speedE;
	float speedN;
	float speedU;
//type of msg
	u8 type;
}GPSDataType;

extern xQueueHandle xUartGPSQueue;
extern xQueueHandle xUartParaQueue;
extern xQueueHandle xUartWayPointQueue;

void vUartRecTask(void *pvParameters);

#endif

