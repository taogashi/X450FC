#ifndef _PIDCTRLER_H_
#define _PIDCTRLER_H_

typedef struct{
	float desired;
	float actual;
	float err;
	float prev_err;
	float deriv;	
	float prev_deriv;
	float integ;
	float i_limit;
	float d_limit;
	float out_limit;
	
	float kp;
	float kd;
	float ki;
	float output;
}PIDCtrlerType;

typedef struct{
	float in;
	float fb;
	float dt;
	unsigned char pid_type_inc;
	void *deriv_filter;
	void *err_filter;
}PIDCtrlerAuxiliaryType;

void ResetCtrler(PIDCtrlerType *ctrler);
void PIDProccessing(PIDCtrlerType *ctrler, PIDCtrlerAuxiliaryType *info);

#endif
