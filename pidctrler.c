#include "pidctrler.h"
#include "filter.h"
#include <stdio.h>

void ResetCtrler(PIDCtrlerType *ctrler)
{
	ctrler->integ = 0.0;
	ctrler->prev_err = 0.0;
	ctrler->output = 0.0;
}

void PIDProccessing(PIDCtrlerType *ctrler, PIDCtrlerAuxiliaryType *info)
{
	ctrler->desired = info->in;
	ctrler->actual = info->fb;
	
	/* increment PID */
	if(info->pid_type == PID_TYPE_INC)
	{
		ctrler->integ = ctrler->desired - ctrler->actual;
		
		ctrler->err = (ctrler->integ - ctrler->prev_err)/info->dt;
		if(info->err_filter != NULL)
			ctrler->err = GaussianFilter((GFilterType *)(info->err_filter), ctrler->err);
		ctrler->prev_err = ctrler->integ;
		
		ctrler->deriv = ctrler->err - ctrler->prev_deriv;
		if(info->deriv_filter != NULL)
			ctrler->deriv = GaussianFilter((GFilterType *)(info->deriv_filter), ctrler->deriv);
		ctrler->prev_deriv = ctrler->deriv;
	}
	/* position PID */
	else
	{
		ctrler->err = ctrler->desired - ctrler->actual;
		if(info->err_filter != NULL)
			ctrler->err = GaussianFilter((GFilterType *)(info->err_filter), ctrler->err);
	
		ctrler->deriv = (ctrler->err - ctrler->prev_err)/info->dt;
		if(info->deriv_filter != NULL)
			ctrler->deriv = GaussianFilter((GFilterType *)(info->deriv_filter), ctrler->deriv);	
		ctrler->prev_err = ctrler->err;
		
		ctrler->integ += ctrler->err*info->dt;
	}
	
	/* amp limit */
	if(ctrler->integ > ctrler->i_limit)
		ctrler->integ = ctrler->i_limit;
	else if(ctrler->integ < -ctrler->i_limit)
		ctrler->integ = -ctrler->i_limit;
	
	if(ctrler->deriv > ctrler->d_limit)
		ctrler->deriv = ctrler->d_limit;
	else if(ctrler->deriv < -ctrler->d_limit)
		ctrler->deriv = -ctrler->d_limit;
	
	ctrler->output = ctrler->kp * ctrler->err 
					+ ctrler->ki * ctrler->integ
					+ ctrler->kd * ctrler->deriv;
					
	if(info->pid_type == PID_TYPE_INC)
	{
		ctrler->output *= info->dt;
	}
	
	if(ctrler->output > ctrler->out_limit)
		ctrler->output = ctrler->out_limit;
	else if(ctrler->output < -ctrler->out_limit)
		ctrler->output = -ctrler->out_limit;
}
