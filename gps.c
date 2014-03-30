#include "gps.h"
#include <arm_math.h>

double init_latitude = 0.0;
double init_longtitude = 0.0;

void GPSSetInitPos(GPSDataType *gdt)
{
	uint16_t temp;

	temp = (uint16_t)(gdt->Lati*0.01);
	init_latitude = 0.01745329*(temp + (gdt->Lati - temp*100.0)*0.0166666667);
	
	temp = (uint16_t)(gdt->Long*0.01);
	init_longtitude = 0.01745329*(temp + (gdt->Long - temp*100.0)*0.0166666667);
}

void GPSSetMag_decline(GPSDataType *gdt, float angle)
{
	gdt->mag_decline = angle;
}

void GPSGetLocalXY(GPSDataType *gdt)
{
	float rawx,rawy;
	float direction;
	
	float sinFi = arm_sin_f32(gdt->mag_decline);
	float cosFi = arm_cos_f32(gdt->mag_decline);
	
	uint16_t temp;
	
	temp = (uint16_t)(gdt->Lati*0.01);
	rawx = (0.01745329 * (temp + (gdt->Lati - temp*100.0)*0.0166666667) - init_latitude)*6371004;
	
	temp = (uint16_t)(gdt->Long*0.01);
	rawy = (0.01745329 * (temp + (gdt->Long - temp*100.0)*0.0166666667) - init_longtitude)*4887077;
	
	gdt->local_pos_N = cosFi * rawx + sinFi * rawy;
	gdt->local_pos_E = -sinFi * rawx + cosFi * rawy;
	
	direction = - gdt->COG*0.01745329 + gdt->mag_decline;
	if(direction < 0.0)
		direction += 2*PI;
	
	gdt->speed_N = gdt->SPD * 0.51444 * arm_cos_f32(direction);
	gdt->speed_E = -gdt->SPD * 0.51444 * arm_sin_f32(direction);
}
