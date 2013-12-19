#ifndef _FLIGHTCONTASK_H_
#define _FLIGHTCONTASK_H_

#include "stm32f4xx.h"

#define ROLL_ANGLE_VALID 	0x01
#define PITCH_ANGLE_VALID 	0x02
#define YAW_ANGLE_VALID		0x04
#define ROLL_RATE_VALID		0x01
#define PITCH_RATE_VALID	0x02
#define YAW_RATE_VALID		0x04

/* index of each parameters in packet */
#define ROLL_RATE_P		0
#define ROLL_RATE_I		1
#define ROLL_RATE_D		2

#define PITCH_RATE_P	3
#define PITCH_RATE_I	4
#define PITCH_RATE_D	5

#define YAW_RATE_P		6
#define YAW_RATE_I		7
#define YAW_RATE_D		8

#define ROLL_P			9
#define ROLL_I			10
#define ROLL_D			11

#define PITCH_P			12
#define PITCH_I			13
#define PITCH_D			14

#define YAW_P			15
#define YAW_I			16
#define YAW_D			17

extern const char* PID_FORMAT_IN;
extern const char* PID_FORMAT_OUT;
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
}OrderType;

typedef struct{
	float roll_moment;
	float pitch_moment;
	float yaw_moment;
	float thrust_out;
}CtrlProcType;

//struct for output command
typedef struct{
	s16 motor1_Out;
	s16 motor2_Out;	
	s16 motor3_Out;	
	s16 motor4_Out;	
}OutputType;

typedef struct{
	float desired;
	float actual;
	float err;
	float prev_err;
	float deriv;	
	float integ;
	float i_limit;
	float d_limit;
	float kp;
	float kd;
	float ki;
	float output;
}PIDCtrlerType;

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

extern OptionalPara optional_param_global;

s32 CalChecksum(OptionalPara* OP);
void vFlyConTask(void* pvParameters);

#endif

