#ifndef _FLIGHTCONTASK_H_
#define _FLIGHTCONTASK_H_

#include "stm32f4xx.h"

extern const char* PARA_FORMAT_IN;
extern const char* PARA_FORMAT_OUT;
extern const char* PID_FORMAT_IN;
extern const char* PID_FORMAT_OUT;
extern const char* NEUTRAL_FORMAT_IN;
extern const char* NEUTRAL_FORMAT_OUT;
extern const char* WAYPOINT_FORMAT_IN;
extern const char* WAYPOINT_FORMAT_OUT;

//struct for attitude control
typedef struct{
	float rollConVal;
	float pitchConVal;
	float yawConVal;

	u8 IntegerEnableOption;
}AttConValType;

//struct for input order
typedef struct{
	float thrustOrder;
	float thrustOut;
	float rollOrder;
	float pitchOrder;
	float yawOrder;
}OrderType;

//struct for output command
typedef struct{
	s16 motor1_Out;
	s16 motor2_Out;	
	s16 motor3_Out;	
	s16 motor4_Out;	
}OutputType;

//optional parameter
typedef struct{
	float rollP;
	float rollD;
	float rollI;
	
	float yawP;
	float yawD1;
	float yawD2;
	float yawI;

	float horiP;
	float horiD;
	float horiI;

	float heightP;
	float heightD;
	float heightI;

	float hoverThrust;

	s16 RCneutral[4];
	
	s16 checksum;
}OptionalPara;

typedef struct{
	u8 properties;	//0:normal
					//1:immediately
					//2:arrived
					//3:ignored
	u8 maxSpeed;	//
	u16 time;	//0.1s/count
	u16 posAcrcy;	//in mm
	int x;		//mm
	int y;		//mm
	int height;	//mm
	int yaw;	//deg
}WayPointType;

s16 CalChecksum(OptionalPara* OP);
void vFlyConTask(void* pvParameters);

#endif

