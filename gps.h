#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

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
	uint16_t satellites_used;	   //??¦Ì?¦Ì??¨¤D?¨ºy
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
	
	float mag_decline;
	
	float local_pos_N;
	float local_pos_E;
	
	float speed_N;
	float speed_E;
	float speed_D;
//type of msg
	uint8_t type;
}GPSDataType;

void GPSSetInitPos(GPSDataType *gdt);
void GPSSetMag_decline(GPSDataType *gdt, float angle);
void GPSGetLocalXY(GPSDataType *gdt);

#endif
