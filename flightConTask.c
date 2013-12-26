#include "flightConTask.h"
#include "AHRSEKF.h"
#include "INSEKF.h"
//#include "sensor.h"
#include "pwm.h"
#include "diskTask.h"
#include "uartTask.h"
#include "ledTask.h"
#include "filter.h"
#include <arm_math.h>
#include <stdio.h>

/* global data*/
const char* PID_FORMAT_IN="PID,%hu,%f,%f,%f,%f,%f,%f,%f,%f,%f";
const char* PID_FORMAT_OUT="set PID para: %d\r\n%.2f,%.2f,%.2f\r\n%.2f,%.2f,%.2f\r\n%.2f,%.2f,%.2f\r\n";
const char* MISCEL_FORMAT_IN = "Miscel,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f";
const char* MISCEL_FORMAT_OUT = "set Miscel,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n";
const char* NEUTRAL_FORMAT_IN="Neutral,%hd,%hd,%hd,%hd";
const char* NEUTRAL_FORMAT_OUT="set Neutral para: %d,%d,%d,%d\r\n";
const char* WAYPOINT_FORMAT_IN="Waypoint,%hd,%hd,%hd,%hd,%d,%d,%d,%d";
const char* WAYPOINT_FORMAT_OUT="Waypoint,%d,%d,%d,%d,%d,%d,%d,%d\r\n";

OptionalPara optional_param_global;

/*********************************** private data************************************/
struct system_level_ctrler{
	//define controllers
	PIDCtrlerType height_ctrler;
	PIDCtrlerType velo_z_ctrler;
	
	PIDCtrlerType roll_ctrler;
	PIDCtrlerType pitch_ctrler;
	PIDCtrlerType yaw_ctrler;
	PIDCtrlerType rollrate_ctrler;
	PIDCtrlerType pitchrate_ctrler;
	PIDCtrlerType yawrate_ctrler;
}system_ctrler;	

GFilterType gaus_filter[8]={
{{0},10,9},//roll rate
{{0},10,9},//pitch rate
{{0},10,9},//yaw rate
{{0},10,9},//roll rate deriv
{{0},10,9},//pitch rate deriv
{{0},10,9},//yaw rate deriv
{{0},10,9},//height
{{0},10,9} //height deriv
};

/************************************ global function **********************************/
s32 CalChecksum(OptionalPara* OP)
{
	s32 sum=0;
	u8 i=0, j;
	for(i=0; i<4; i++)
	{
		for(j=0; j<3; j++)
			sum += (s16)(OP->loop_pid[i].xPID[j] + OP->loop_pid[i].yPID[j] + OP->loop_pid[i].zPID[j]);
	}
	for(i=0; i<10; i++)
	{
		sum += (s16)(OP->miscel[i]);
	}
	for(i=0; i<4; i++)
	{
		sum += OP->RCneutral[i];
	}
	return sum;
}


/************************************* private function ***************************/
void LoadParam(void)
{
	char print_buffer[100];
	u16 string_len;
	portBASE_TYPE param_read_status, xstatus;
	u16 index;
	u8 i;
	
	string_len = sprintf(print_buffer, "load para...\r\n");
	UartSend(print_buffer, string_len);
	param_read_status = xQueueReceive(xDiskParamQueue,&index,(portTickType)(5000/portTICK_RATE_MS));
	if(param_read_status == pdPASS)
	{
		string_len = sprintf(print_buffer, "OK!\r\n");
		UartSend(print_buffer, string_len);
		for(i=0; i<3; i++)
		{	
			string_len = sprintf(print_buffer,"%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f\r\n"
								,optional_param_global.loop_pid[0].xPID[i],optional_param_global.loop_pid[0].yPID[i],optional_param_global.loop_pid[0].zPID[i]
								,optional_param_global.loop_pid[1].xPID[i],optional_param_global.loop_pid[1].yPID[i],optional_param_global.loop_pid[1].zPID[i]
								,optional_param_global.loop_pid[2].xPID[i],optional_param_global.loop_pid[2].yPID[i],optional_param_global.loop_pid[2].zPID[i]
								,optional_param_global.loop_pid[3].xPID[i],optional_param_global.loop_pid[3].yPID[i],optional_param_global.loop_pid[3].zPID[i]);
			UartSend(print_buffer, string_len);
			vTaskDelay((portTickType)(300/portTICK_RATE_MS));
		}
		Blinks(LED1,1);
	}
	//if fails, read from uart
	else
	{
		u16 param_req=0;
		string_len = sprintf(print_buffer,"failed to read disk, checking uart...\r\n");
		UartSend(print_buffer, string_len);
		while(param_read_status != pdPASS)
		{
			xstatus = xQueueReceive(xUartParaQueue,&index,0);
			if(xstatus == pdPASS)
			{
				if(index == param_req)
				{
					param_req ++;
				}
				if(param_req == 2)
				{
					string_len = sprintf(print_buffer, "minimal parameters received\r\n");
					UartSend(print_buffer, string_len);
					optional_param_global.checksum = CalChecksum(&optional_param_global);
					xQueueSend(xDiskParamQueue, &index, 0);//write to SD card
					param_read_status = pdPASS;
				}				
			}
			vTaskDelay((portTickType)(20/portTICK_RATE_MS));
		}
		Blinks(LED1,1);
	}	
}
/*
note:
youmen 		capture from TIM4_3
pianhang 	capture from TIM4_4
fuyang		capture from TIM4_2
gunzhuan	capture from TIM4_1
*/
/*作用：从接收机获取输入的姿态控制指令
 * 参数：指令结构体指针
 * 返回值：无*/
void InputControl(OrderType* odt)
{
	odt->thrustOrder = (tim4IC3Width-optional_param_global.RCneutral[2])*0.00125;	//归一化
	odt->yawOrder=(tim4IC4Width-optional_param_global.RCneutral[3])*0.003;	//折算成角速度指令	 最大30°/s
	odt->pitchOrder=(tim4IC2Width-optional_param_global.RCneutral[1])*0.002;	//折算成角度指令	 最大30°
	odt->rollOrder=(tim4IC1Width-optional_param_global.RCneutral[0])*0.002;	//折算成角度指令 最大30°
	
	//vertical
	if(tim5IC2Width > 1500)
		odt->hover_en = 0;
	else
		odt->hover_en = 1;
	
	//horizontal
	if(tim5IC1Width > 1500)
		odt->lock_en = 0;
	else
		odt->lock_en = 1;
	
	if(odt->thrustOrder < 0.00001)
		odt->thrustOrder = 0.0;
	else if(odt->thrustOrder > 1.0)
		odt->thrustOrder = 1.0;
}

void ControllerInit(void)
{
	memset((void *)(&system_ctrler), 0, sizeof(system_ctrler));
	
	system_ctrler.rollrate_ctrler.kp = optional_param_global.loop_pid[0].xPID[0];
	system_ctrler.rollrate_ctrler.ki = optional_param_global.loop_pid[0].xPID[1];
	system_ctrler.rollrate_ctrler.kd = optional_param_global.loop_pid[0].xPID[2];
	system_ctrler.rollrate_ctrler.i_limit = 0.6;
	system_ctrler.rollrate_ctrler.d_limit = 15.0;
	
	system_ctrler.pitchrate_ctrler.kp = optional_param_global.loop_pid[0].yPID[0];
	system_ctrler.pitchrate_ctrler.ki = optional_param_global.loop_pid[0].yPID[1];
	system_ctrler.pitchrate_ctrler.kd = optional_param_global.loop_pid[0].yPID[2];
	system_ctrler.pitchrate_ctrler.i_limit = 0.6;
	system_ctrler.pitchrate_ctrler.d_limit = 1.0;
	
	system_ctrler.yawrate_ctrler.kp = optional_param_global.loop_pid[0].zPID[0];
	system_ctrler.yawrate_ctrler.ki = optional_param_global.loop_pid[0].zPID[1];
	system_ctrler.yawrate_ctrler.kd = optional_param_global.loop_pid[0].zPID[2];
	system_ctrler.yawrate_ctrler.i_limit = 0.6;
	system_ctrler.yawrate_ctrler.d_limit = 1.0;
	
	system_ctrler.roll_ctrler.kp = optional_param_global.loop_pid[1].xPID[0];
	system_ctrler.roll_ctrler.ki = optional_param_global.loop_pid[1].xPID[1];
	system_ctrler.roll_ctrler.kd = optional_param_global.loop_pid[1].xPID[2];
	system_ctrler.roll_ctrler.i_limit = 0.6;
	system_ctrler.roll_ctrler.d_limit = 1.0;
	
	system_ctrler.pitch_ctrler.kp = optional_param_global.loop_pid[1].yPID[0];
	system_ctrler.pitch_ctrler.ki = optional_param_global.loop_pid[1].yPID[1];
	system_ctrler.pitch_ctrler.kd = optional_param_global.loop_pid[1].yPID[2];
	system_ctrler.pitch_ctrler.i_limit = 0.6;
	system_ctrler.pitch_ctrler.d_limit = 1.0;
	
	system_ctrler.yaw_ctrler.kp = optional_param_global.loop_pid[1].zPID[0];
	system_ctrler.yaw_ctrler.ki = optional_param_global.loop_pid[1].zPID[1];
	system_ctrler.yaw_ctrler.kd = optional_param_global.loop_pid[1].zPID[2];
	system_ctrler.yaw_ctrler.i_limit = 0.6;
	system_ctrler.yaw_ctrler.d_limit = 1.0;
	
	system_ctrler.velo_z_ctrler.kp = optional_param_global.loop_pid[2].zPID[0];
	system_ctrler.velo_z_ctrler.ki = optional_param_global.loop_pid[2].zPID[1];
	system_ctrler.velo_z_ctrler.kd = optional_param_global.loop_pid[2].zPID[2];
	system_ctrler.velo_z_ctrler.i_limit = 1.0;
	system_ctrler.velo_z_ctrler.d_limit = 5.0;
	
	system_ctrler.height_ctrler.kp = optional_param_global.loop_pid[3].zPID[0];
	system_ctrler.height_ctrler.ki = optional_param_global.loop_pid[3].zPID[1];
	system_ctrler.height_ctrler.kd = optional_param_global.loop_pid[3].zPID[2];
	system_ctrler.height_ctrler.i_limit = 1.0;
	system_ctrler.height_ctrler.d_limit = 4.0;
}


/* index: 
	0 rate loop
	1 angle loop
	2 velocity loop
	3 position loop
*/
void ControllerUpdate(u16 index)
{
	switch(index)
	{
		/* if pid parameters update, update controllers */
		case 0:
			system_ctrler.rollrate_ctrler.kp = optional_param_global.loop_pid[0].xPID[0];
			system_ctrler.rollrate_ctrler.ki = optional_param_global.loop_pid[0].xPID[1];
			system_ctrler.rollrate_ctrler.kd = optional_param_global.loop_pid[0].xPID[2];
		
			system_ctrler.pitchrate_ctrler.kp = optional_param_global.loop_pid[0].yPID[0];
			system_ctrler.pitchrate_ctrler.ki = optional_param_global.loop_pid[0].yPID[1];
			system_ctrler.pitchrate_ctrler.kd = optional_param_global.loop_pid[0].yPID[2];
		
			system_ctrler.yawrate_ctrler.kp = optional_param_global.loop_pid[0].zPID[0];
			system_ctrler.yawrate_ctrler.ki = optional_param_global.loop_pid[0].zPID[1];
			system_ctrler.yawrate_ctrler.kd = optional_param_global.loop_pid[0].zPID[2];
			
			break;
		
		case 1:
			system_ctrler.roll_ctrler.kp = optional_param_global.loop_pid[1].xPID[0];
			system_ctrler.roll_ctrler.ki = optional_param_global.loop_pid[1].xPID[1];
			system_ctrler.roll_ctrler.kd = optional_param_global.loop_pid[1].xPID[2];
		
			system_ctrler.pitch_ctrler.kp = optional_param_global.loop_pid[1].yPID[0];
			system_ctrler.pitch_ctrler.ki = optional_param_global.loop_pid[1].yPID[1];
			system_ctrler.pitch_ctrler.kd = optional_param_global.loop_pid[1].yPID[2];
		
			system_ctrler.yaw_ctrler.kp = optional_param_global.loop_pid[1].zPID[0];
			system_ctrler.yaw_ctrler.ki = optional_param_global.loop_pid[1].zPID[1];
			system_ctrler.yaw_ctrler.kd = optional_param_global.loop_pid[1].zPID[2];
			break;
		
		case 2:
			system_ctrler.velo_z_ctrler.kp = optional_param_global.loop_pid[2].zPID[0];
			system_ctrler.velo_z_ctrler.ki = optional_param_global.loop_pid[2].zPID[1];
			system_ctrler.velo_z_ctrler.kd = optional_param_global.loop_pid[2].zPID[2];
			
			break;
		case 3:
			system_ctrler.height_ctrler.kp = optional_param_global.loop_pid[3].zPID[0];
			system_ctrler.height_ctrler.ki = optional_param_global.loop_pid[3].zPID[1];
			system_ctrler.height_ctrler.kd = optional_param_global.loop_pid[3].zPID[2];
			
			break;
		default:
			break;
	}
	optional_param_global.checksum = CalChecksum(&optional_param_global);
	xQueueSend(xDiskParamQueue, &index, 0);	
}

void PIDProccessing(PIDCtrlerType *ctrler, float in, float fb, float dt, GFilterType *d_filter, GFilterType *filter)
{
	ctrler->desired = in;
	ctrler->actual = fb;
	
	ctrler->err = ctrler->desired - ctrler->actual;
	if(filter != NULL)
		ctrler->err = GaussianFilter(filter, ctrler->err);
	
	ctrler->deriv = (ctrler->err - ctrler->prev_err)/dt;
	if(d_filter != NULL)
		ctrler->deriv = GaussianFilter(d_filter, ctrler->deriv);
	
	ctrler->prev_err = ctrler->err;
	ctrler->integ += ctrler->err*dt;
	
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
}

/* 作用：根据当前的姿态和姿态变化率，结合输入的姿态控制指令，计算应当输出的控制量
 * 参数：
 * 	acdt
 * 	结构体指针，包含姿态角、姿态角速率信息
 * 	acvt
 * 	指向姿态控制结构体的指针
 * 	odt
 * 	指向指令结构体的指针
 * 返回值：无*/
//void AttControl(AttConDataType *acdt,AttConValType* acvt,OrderType* odt,OptionalPara* OP)
//{
//	/*有人操作情况下的PID*/
//	//遥控器输入命令为偏航角速度、滚转角、俯仰角。
//	//三者的控制为有静差控制（需要有人操作）。没有偏航指令时PI控制，保持方向、偏航角无静差
//	//偏航用P调节
//	float yawD;	
//	float roll_e1=0.0;	
//	float nick_e1=0.0;

//	static float rollErrInteg=0.0;
//	static float nickErrInteg=0.0;

//	float yaw_err=0.0;
//	static float yawErrInteg=0.0;
//	static float curHeading=0.0;	   //当前的机头指向

////-------------------------------------------roll---------------------------------------------													
//	roll_e1 = acdt->rollAngle - odt->rollOrder;		//误差				
//	rollErrInteg+= roll_e1;
//	if(acvt->IntegerEnableOption==0)			   //
//	{
//		rollErrInteg=0;
//	}
//	else if(rollErrInteg<-80.0)
//	{
//		rollErrInteg=-80.0;
//	}
//	else if(rollErrInteg>80.0)
//	{
//		rollErrInteg=80.0;
//	}

//	acvt->rollConVal = OP->rollP*(roll_e1) + OP->rollD*(acdt->rollAngleRate) + OP->rollI*rollErrInteg;			// PID调节器	rollstate量纲是角度

////-----------------------------------------------nick------------------------------------
//	nick_e1 = acdt->pitchAngle-odt->pitchOrder;
//	nickErrInteg+= nick_e1;
//	if(acvt->IntegerEnableOption==0)
//	{
//		nickErrInteg=0;
//	}
//	else if(nickErrInteg<-80.0)
//	{
//		nickErrInteg=-80.0;
//	}
//	else if(nickErrInteg>80.0)
//	{
//		nickErrInteg=80.0;
//	}
//	acvt->pitchConVal = OP->rollP*(nick_e1) + OP->rollD*(acdt->pitchAngleRate)+ OP->rollI*nickErrInteg;	 //PD调节器//	

////---------------------------------------------------------yaw----------------------------------------------
//	if(odt->yawOrder>-0.03 && odt->yawOrder<0.03)	//此时认为遥控器无输入
//	{
//		yawD=CalYawD(0.1,OP->yawD1,0.8,OP->yawD2,acdt->yawAngleRate);

//		yaw_err=acdt->yawAngle-curHeading;
//		if(yaw_err > 3.1415926)
//			yaw_err = yaw_err - 3.1415926*2.0;
//		else if(yaw_err < -3.1415926)
//			yaw_err = yaw_err + 3.1415926*2.0;
//		
//		yawErrInteg+= yaw_err;
//		if(acvt->IntegerEnableOption==0)			   //
//		{
//			yawErrInteg=0;
//		}
//		else if(yawErrInteg<-100.0)
//		{
//			yawErrInteg=-100.0;
//		}
//		else if(yawErrInteg>100.0)
//		{
//			yawErrInteg=100.0;
//		}
//		acvt->yawConVal=OP->yawP*yaw_err+yawD*acdt->yawAngleRate+OP->yawI*yawErrInteg;	   //	   
//	}
//	else
//	{	
//		curHeading=acdt->yawAngle;
//		yaw_err=acdt->yawAngleRate-odt->yawOrder;
//		acvt->yawConVal=(OP->yawD1+OP->yawD2)*1.2*yaw_err;
//	}
//}

//void PosControl(OrderType* odt,PosConDataType* pcdt,AttConDataType *acdt,OptionalPara* OP)//
//{
//	static float targetXPos=0.0;
//	static float targetYPos=0.0;
//	float xPosErr=0.0;
//	float yPosErr=0.0;
//	static float xPosErrIntg=0.0;
//	static float yPosErrIntg=0.0;

//	float tarHeight=odt->thrustOrder*20.0;
//	float heightErr=0.0;
//	static float heightErrInteg=0.0;

//	float xPID,yPID;
//	
////	char printf_buffer[100];
//	//高度控制
////	if(odt->thrustOrder<0.05) odt->thrustOut=0;
////	else
////	{
////		//高度误差
////		heightErr = pcdt->posZ-tarHeight;
////		//高度误差积分
////		heightErrInteg+=heightErr*0.005; 
////		//积分限幅
////		if(heightErrInteg>1.0) heightErrInteg=1.0;
////		if(heightErrInteg<-1.0) heightErrInteg=-1.0;
////	
////		odt->thrustOut=OP->hoverThrust - OP->heightP*heightErr + OP->heightD*pcdt->veloZ - OP->heightI*heightErrInteg;
////	}
////
////	if(odt->thrustOut>1) odt->thrustOut=1;
////	else if(odt->thrustOut<0) odt->thrustOut=0;

////	if(pcdt->veloZ<0.03 && pcdt->veloZ>-0.03 && odt->thrustOut>0.4) OP->hoverThrust=0.99*OP->hoverThrust+0.01*odt->thrustOut;

//	//水平位置控制
//	if(tim5IC1Width>1500)
//	{
//		targetXPos=pcdt->posX;
//		targetYPos=pcdt->posY;
//		xPosErrIntg = 0.0;
//		yPosErrIntg = 0.0;
//	}
//	else
//	{
//		if((odt->rollOrder<0.02 && odt->rollOrder>-0.02) && (odt->pitchOrder<0.02 && odt->pitchOrder>-0.02))
//		{
//			float cosfi = arm_cos_f32(acdt->yawAngle);
//			float sinfi = arm_sin_f32(acdt->yawAngle);	
//			xPosErr=pcdt->posX-targetXPos;
//			yPosErr=pcdt->posY-targetYPos;
//			xPosErrIntg += xPosErr*0.01;
//			if(xPosErrIntg > 50.0) xPosErrIntg=50.0;
//			else if(xPosErrIntg <-50.0) xPosErrIntg=-50.0;
//			yPosErrIntg += yPosErr;
//			if(yPosErrIntg > 50.0) yPosErrIntg=50.0;
//			else if(yPosErrIntg <-50.0) yPosErrIntg=-50.0;
//			
//			/*PID*/
//			xPID = OP->horiP*xPosErr + OP->horiD*pcdt->veloX + OP->horiI*xPosErrIntg;
//			yPID = OP->horiP*yPosErr + OP->horiD*pcdt->veloY + OP->horiI*yPosErrIntg;
//			
//			/*decouple*/
//			odt->pitchOrder = 0.03*(xPID*cosfi+yPID*sinfi);
//			if(odt->pitchOrder > 0.18) odt->pitchOrder=0.18;
//			else if(odt->pitchOrder < -0.18) odt->pitchOrder=-0.18;

//			odt->rollOrder = -0.03*(-xPID*sinfi+yPID*cosfi);
//			if(odt->rollOrder > 0.18) odt->rollOrder=0.18;
//			else if(odt->rollOrder < -0.18) odt->rollOrder=-0.18;
//			
////			sprintf(printf_buffer,"%.2f %.2f\r\n",xPosErr,yPosErr);
////			UartSend(printf_buffer,strlen(printf_buffer));
//		}	
//		else
//		{
//			targetXPos=pcdt->posX;
//			targetYPos=pcdt->posX;
//			xPosErrIntg=0.0;
//			yPosErrIntg=0.0;
//		}	
//	}
//	odt->thrustOut=odt->thrustOrder;
//}

/* 作用：根据acvt指令和姿态控制量，计算输出给四个电机的转速指令
 * 参数：
 * 	odt
 * 	opt
 * 返回值：无*/
void OutputControl(CtrlProcType *cpt, OutputType* opt)
{
	s16 youmenOut=200+(s16)(cpt->thrust_out*800);
	if(youmenOut<250) youmenOut=100;
	
	opt->motor1_Out = youmenOut + cpt->roll_moment + cpt->pitch_moment - cpt->yaw_moment;
	opt->motor2_Out = youmenOut + cpt->roll_moment - cpt->pitch_moment + cpt->yaw_moment;
	opt->motor3_Out = youmenOut - cpt->roll_moment - cpt->pitch_moment - cpt->yaw_moment;
	opt->motor4_Out = youmenOut - cpt->roll_moment + cpt->pitch_moment + cpt->yaw_moment;

	if(opt->motor1_Out<250) 
		opt->motor1_Out=100;
	else if(opt->motor1_Out>1100) 
		opt->motor1_Out=1100; 

	if(opt->motor2_Out<250) 
		opt->motor2_Out=100;
	else if(opt->motor2_Out>1100) 
		opt->motor2_Out=1100;

	if(opt->motor3_Out<250) 
		opt->motor3_Out=100;
	else if(opt->motor3_Out>1100) 
		opt->motor3_Out=1100;

	if(opt->motor4_Out<250) 
		opt->motor4_Out=100;
	else if(opt->motor4_Out>1100) 
		opt->motor4_Out=1100;
	
	if(cpt->thrust_out<0.05)
	{
		opt->motor1_Out = 100;
		opt->motor2_Out = 100;
		opt->motor3_Out = 100;
		opt->motor4_Out = 100;
	}
}

/* 作用：输出指令控制电机转速
 * 参数：opt
 * 返回值：无*/
void WriteMotor(OutputType* opt)
{
	TIM_SetCompare1(TIM3,opt->motor1_Out);	//youmenOut 	 
	TIM_SetCompare2(TIM3,opt->motor2_Out);	//youmenOut 	  
	TIM_SetCompare3(TIM3,opt->motor3_Out);	//youmenOut	  
	TIM_SetCompare4(TIM3,opt->motor4_Out);	//youmenOut	

//	TIM_SetCompare1(TIM3,100);	//youmenOut 	 
//	TIM_SetCompare2(TIM3,100);	//youmenOut 	  
//	TIM_SetCompare3(TIM3,100);	//youmenOut	  
//	TIM_SetCompare4(TIM3,100);	//youmenOut	
}	

void vFlyConTask(void* pvParameters)
{
	//for print
	char printf_buffer[100];
	u16 string_len;
	u8 CNT=0;
	
	//for queue receive
	portBASE_TYPE xstatus;
	u16 index;
	
	float height_locked = 0.0;
	float yaw_angle_locked = 0.0;
	AHRSDataType adt;
	PosDataType pdt;
	
	//orders from remote controller
	OrderType odt;
	
	//feed back from sensor system
	FeedBackValType fbvt;
	
	//intermediate quantity in controllers
	CtrlProcType cpt;
	
	//final thrust output to each motor
	OutputType opt={100,100,100,100};

	portTickType lastTime;

	/******************* initialize *************************************/
	xQueueReceive(AHRSToFlightConQueue,&adt,portMAX_DELAY);
	yaw_angle_locked = adt.yawAngle;
	optional_param_global.RCneutral[0] = 1500;
	optional_param_global.RCneutral[1] = 1500;
	optional_param_global.RCneutral[2] = 1100;
	optional_param_global.RCneutral[3] = 1500;
	optional_param_global.miscel[0] = 0.48;
	
	/****************** parameters read form disk ***********************/
	LoadParam();
	
	/****************** init controllers ************************/
	ControllerInit();
	
	/******************* enable PWM width capture ***************/
	TIM4_IT_Config();
	TIM5_IT_Config();
	
	lastTime = xTaskGetTickCount();

	for(;;)
	{
		/* handle uart parameters */
		xstatus = xQueueReceive(xUartParaQueue,&index,0);
		if(xstatus == pdPASS)
		{
			ControllerUpdate(index);
		}

		/************* update attitude data *************************/
		xQueueReceive(AHRSToFlightConQueue,&adt,portMAX_DELAY);
		/************* update pos data ******************************/
		xstatus = xQueueReceive(INSToFlightConQueue, &pdt, 0);
		
		/************* feed back *************************/
		if(xstatus == pdPASS)
		{
			fbvt.pos_x = pdt.posX;
			fbvt.pos_y = pdt.posY;
			fbvt.pos_z = pdt.posZ;
			fbvt.pos_valid = POS_Z_VALID;
			
			fbvt.velo_x = pdt.veloX;
			fbvt.velo_y = pdt.veloY;
			fbvt.velo_z = pdt.veloZ;
			fbvt.velo_valid = VELO_Z_VALID;
		}
		else
		{
			fbvt.pos_valid = 0;
			fbvt.velo_valid = 0;
		}	
		
		fbvt.roll_angle = adt.rollAngle;
		fbvt.pitch_angle = adt.pitchAngle;
		fbvt.yaw_angle = adt.yawAngle;
		fbvt.angle_valid = ROLL_ANGLE_VALID | PITCH_ANGLE_VALID | YAW_ANGLE_VALID;
		
		fbvt.roll_rate = adt.rollAngleRate;
		fbvt.pitch_rate = adt.pitchAngleRate;
		fbvt.yaw_rate = adt.yawAngleRate;
		fbvt.rate_valid = ROLL_RATE_VALID | PITCH_RATE_VALID | YAW_RATE_VALID;

		/************* input *************************/
		InputControl(&odt);

		/************* controllers *************************/
		
		/*vertical loop*/
		if(odt.hover_en == 1)
		{
			if((fbvt.pos_valid & POS_Z_VALID) != 0)
				PIDProccessing(&(system_ctrler.height_ctrler)
								, height_locked
								, fbvt.pos_z
								, 0.005
								, NULL
								, NULL);
			if((fbvt.velo_valid & VELO_Z_VALID) != 0)
				PIDProccessing(&(system_ctrler.velo_z_ctrler)
								, system_ctrler.height_ctrler.output
								, -fbvt.velo_z
								, 0.005
								, NULL
								, NULL);
			cpt.thrust_out = optional_param_global.miscel[0] + system_ctrler.velo_z_ctrler.output; 
		}
		else
		{
			height_locked = fbvt.pos_z;
			cpt.thrust_out = odt.thrustOrder;
		}
		
		/*roll loop*/
		if((fbvt.angle_valid & ROLL_ANGLE_VALID) != 0)
			PIDProccessing(&(system_ctrler.roll_ctrler)
							, odt.rollOrder
							, fbvt.roll_angle
							, 0.005
							, NULL
							, NULL);
	
		if((fbvt.rate_valid & ROLL_RATE_VALID) != 0)
			PIDProccessing(&(system_ctrler.rollrate_ctrler)
							, system_ctrler.roll_ctrler.output
							, fbvt.roll_rate
							, 0.005
							, &(gaus_filter[0])
							, &(gaus_filter[3]));
		
		/*pitch loop*/
		if((fbvt.angle_valid & PITCH_ANGLE_VALID) != 0)
			PIDProccessing(&(system_ctrler.pitch_ctrler)
							, odt.pitchOrder
							, fbvt.pitch_angle
							, 0.005
							, NULL
							, NULL);
		if((fbvt.rate_valid & PITCH_RATE_VALID) != 0)
			PIDProccessing(&(system_ctrler.pitchrate_ctrler)
							, system_ctrler.pitch_ctrler.output
							, fbvt.pitch_rate
							, 0.005
							, &(gaus_filter[1])
							, &(gaus_filter[4]));
		
		/*yaw loop*/
		if(odt.yawOrder<0.05 && odt.yawOrder>-0.05 && (fbvt.angle_valid & YAW_ANGLE_VALID) != 0)
		{
			if(yaw_angle_locked - fbvt.yaw_angle > PI)
				fbvt.yaw_angle += 2*PI;
			else if(yaw_angle_locked - fbvt.yaw_angle < -PI)
				fbvt.yaw_angle -= 2*PI;
			PIDProccessing(&(system_ctrler.yaw_ctrler)
							, yaw_angle_locked
							, fbvt.yaw_angle
							, 0.005
							, NULL
							, NULL);		
			
			if((fbvt.rate_valid & YAW_RATE_VALID) != 0)
				PIDProccessing(&(system_ctrler.yawrate_ctrler)
							, system_ctrler.yaw_ctrler.output
							, fbvt.yaw_rate
							, 0.005
							, &(gaus_filter[2])
							, &(gaus_filter[5]));
		}	
		else if((fbvt.rate_valid & YAW_RATE_VALID) != 0)
		{
			yaw_angle_locked = fbvt.yaw_angle;
			PIDProccessing(&(system_ctrler.yawrate_ctrler)
							, odt.yawOrder
							, fbvt.yaw_rate
							, 0.005
							, &(gaus_filter[2])
							, &(gaus_filter[5]));
		}
		
		cpt.roll_moment = system_ctrler.rollrate_ctrler.output;
		cpt.pitch_moment = system_ctrler.pitchrate_ctrler.output;
		cpt.yaw_moment = system_ctrler.yawrate_ctrler.output;//0.0;
		
		/************* decouple	 *************************/
		OutputControl(&cpt, &opt);
		WriteMotor(&opt);

		/************ print message **********************/
		if(CNT++>=30)
		{
			CNT=0;
//			sprintf(printf_buffer,"%.4f %.4f %.4f %.4f\r\n",system_ctrler.rollrate_ctrler.err, system_ctrler.rollrate_ctrler.integ, system_ctrler.rollrate_ctrler.deriv, system_ctrler.rollrate_ctrler.output);
//			sprintf(printf_buffer,"%d %d %d %d\r\n",opt.motor1_Out, opt.motor2_Out, opt.motor3_Out, opt.motor4_Out);
//			string_len = sprintf(printf_buffer, "%.2f %.2f\r\n", adt.pitchAngle*57.3, adt.pitchAngleRate*57.3);
			string_len = sprintf(printf_buffer, "%.2f %.2f %.2f %.2f %.2f\r\n", fbvt.pos_z
						, fbvt.velo_z
						, system_ctrler.height_ctrler.output
						, system_ctrler.velo_z_ctrler.output
						, cpt.thrust_out);
			UartSend(printf_buffer,string_len);
		}
		vTaskDelayUntil(&lastTime,(portTickType)(5/portTICK_RATE_MS));
	}
}
