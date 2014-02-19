#ifndef _FLIGHTCONTASK_H_
#define _FLIGHTCONTASK_H_

#include "stm32f4xx.h"

#define POS_LOOP_DIVIDER	16
#define VELO_LOOP_DIVIDER	8
#define ANGLE_LOOP_DIVIDER 	4

#define POS_X_VALID		(u8)0x01
#define POS_Y_VALID		(u8)0x02
#define POS_Z_VALID		(u8)0x04
#define POS_ALL_VALID	(u8)0x07

#define VELO_X_VALID		(u8)0x01
#define VELO_Y_VALID		(u8)0x02
#define VELO_Z_VALID		(u8)0x04
#define VELO_ALL_VALID		(u8)0x07

#define ROLL_ANGLE_VALID 	(u8)0x01
#define PITCH_ANGLE_VALID 	(u8)0x02
#define YAW_ANGLE_VALID		(u8)0x04
#define ANGLE_ALL_VALID		(u8)0x07

#define ROLL_RATE_VALID		(u8)0x01
#define PITCH_RATE_VALID	(u8)0x02
#define YAW_RATE_VALID		(u8)0x04
#define RATE_ALL_VALID		(u8)0x07

extern const char* PID_FORMAT_IN;
extern const char* PID_FORMAT_OUT;
extern const char* MISCEL_FORMAT_IN;
extern const char* MISCEL_FORMAT_OUT;
extern const char* NEUTRAL_FORMAT_IN;
extern const char* NEUTRAL_FORMAT_OUT;
extern const char* WAYPOINT_FORMAT_IN;
extern const char* WAYPOINT_FORMAT_OUT;

typedef struct{
	float xPID[3];
	float yPID[3];
	float zPID[3];
}LoopPIDParam;

//optional parameter
typedef struct{
	LoopPIDParam loop_pid[4];
	float miscel[10];
	s16 RCneutral[4];
	s32 checksum;
}OptionalPara;

//struct for attitude control
typedef struct{
	float pos_x;
	float pos_y;
	float pos_z;
	u8 pos_valid;
	
	float velo_x;
	float velo_y;
	float velo_z;
	u8 velo_valid;
	
	float roll_angle;
	float pitch_angle;
	float yaw_angle;
	u8 angle_valid;
	
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	u8 rate_valid;
}FeedBackValType;

//struct for input order
typedef struct{
	float thrustOrder;
	float rollOrder;
	float pitchOrder;
	float yawOrder;
	
	u8 hover_en;
	u8 lock_en;
}OrderType;

//0.0~1.0
typedef struct{
	float roll_moment;
	float pitch_moment;
	float yaw_moment;
	float thrust_out;
}CtrlProcType;

//struct for output command
typedef struct{
	float motor1_Out;
	float motor2_Out;	
	float motor3_Out;	
	float motor4_Out;	
}OutputType;

typedef struct{
	u8 properties;	//0:normal
					//1:immediately
					//2:arrived
					//3:ignored
	u8 maxSpeed;	//
	u16 time;	//0.1s/count
	u16 posAcrcy;	//in mm
	u16 yaw;	//1/4000 rad
	int x;		//mm
	int y;		//mm
	int height;	//mm
}WayPointType;

extern OptionalPara optional_param_global;

s32 CalChecksum(OptionalPara* OP);
void vFlyConTask(void* pvParameters);

#endif

