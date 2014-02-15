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

/********************************** global data **************************/
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
	PIDCtrlerType px_ctrler;
	PIDCtrlerType py_ctrler;
	PIDCtrlerType height_ctrler;
	
	PIDCtrlerType velo_x_ctrler;
	PIDCtrlerType velo_y_ctrler;
	PIDCtrlerType velo_z_ctrler;
	
	PIDCtrlerType roll_ctrler;
	PIDCtrlerType pitch_ctrler;
	PIDCtrlerType yaw_ctrler;
	
	PIDCtrlerType rollrate_ctrler;
	PIDCtrlerType pitchrate_ctrler;
	PIDCtrlerType yawrate_ctrler;
}system_ctrler;	

//define low pass filter for each PID controller
GFilterType roll_rate_lpf = {{0},10,9};
GFilterType pitch_rate_lpf = {{0},10,9};
GFilterType yaw_rate_lpf = {{0},10,9};
GFilterType droll_rate_lpf = {{0},10,9};
GFilterType dpitch_rate_lpf = {{0},10,9};
GFilterType dyaw_rate_lpf = {{0},10,9};
GFilterType height_lpf = {{0},10,9};
GFilterType dheight_lpf = {{0},10,9};

/************************************* private function ***************************/
void LoadParam(void);
void InputControl(OrderType* odt);
void ControllerInit(void);
void ControllerUpdate(u16 index);
void ResetCtrler(PIDCtrlerType *ctrler);
void PIDProccessing(PIDCtrlerType *ctrler, PIDCtrlerAuxiliaryType *info);
void Pos2AngleMixer(float xPID, float yPID, OrderType *odt, float yawAngle);
void OutputControl(CtrlProcType *cpt, OutputType* opt);
void WriteMotor(OutputType* opt);
void WaitRCSignal(void);

void FeedBack(FeedBackValType *fbvt, AHRSDataType *adt, PosDataType *pdt, portBASE_TYPE pos_data_valid);
void PosLoop(FeedBackValType *fbvt, WayPointType *wpt, struct system_level_ctrler *system_ctrler, float dt);
void HorVeloLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt);
void HeightLoop(FeedBackValType *fbvt, WayPointType *wpt, struct system_level_ctrler *system_ctrler, float dt);
void HeightVeloLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt);
void AngleLoop(FeedBackValType *fbvt, OrderType *odt, float *desired_yaw, struct system_level_ctrler *system_ctrler, float dt);
void RateLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt);

void vFlyConTask(void* pvParameters)
{
	//for print
	char printf_buffer[100];
	u16 string_len;
	u8 CNT=0;
	
	u8 pos_loop_cnt = 0;
	u8 velo_loop_cnt = 0;
	u8 angle_loop_cnt = 0;
	
	//for queue receive
	portBASE_TYPE xstatus;
	u16 index;
	
	// currently way point is used to record position,
	// thus vehicle can hover at a known point
	WayPointType wpt = {
		0		//normal
		,255	//maxSpeed
		,65535	//max stay time
		,200	//200mm position accuracy
		,0		//yaw
		,0		//x
		,0		//y
		,0};	//height
	
	////feed back from sensor system
	AHRSDataType adt;
	PosDataType pdt;	
	FeedBackValType fbvt;
	
	//intermediate quantity in controllers
	CtrlProcType cpt;
	
	//orders from remote controller
	OrderType odt;
	
	float yaw_locked;

	//final thrust output to each motor
	OutputType opt={0.0,0.0,0.0,0.0};

	portTickType lastTime;

	Blinks(LED1,4);
	/******************* initialize *************************************/
	xQueueReceive(AHRSToFlightConQueue,&adt,portMAX_DELAY);
	
	yaw_locked = adt.yawAngle;
	
	optional_param_global.RCneutral[0] = 1500;
	optional_param_global.RCneutral[1] = 1500;
	optional_param_global.RCneutral[2] = 1100;
	optional_param_global.RCneutral[3] = 1500;
	optional_param_global.miscel[0] = 0.46;
	
	/****************** parameters read form disk ***********************/
	LoadParam();
	
	/****************** init controllers ************************/
	ControllerInit();
	
	/******************* enable PWM width capture ***************/
	TIM4_IT_Config();
	TIM5_IT_Config();
	
	/******************* wait signal ****************************/
	Blinks(LED1,3);
	WaitRCSignal();
	
	Blinks(LED1,2);
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
		FeedBack(&fbvt, &adt, &pdt, xstatus);

		/************* input *************************/
		InputControl(&odt);

		/************* controllers *************************/

		
		/****************** position loop **********************/
		if(pos_loop_cnt++ >= POS_LOOP_DIVIDER)
		{
			if((fbvt.pos_valid & POS_X_VALID)!=0 && (fbvt.pos_valid & POS_Y_VALID)!=0)
			{
				/*position control*/
				PosLoop(&fbvt, &wpt, &system_ctrler, 0.005);
				pos_loop_cnt = 0;
				fbvt.pos_valid &= (~POS_X_VALID);
				fbvt.pos_valid &= (~POS_Y_VALID);
			}
			
			if((fbvt.pos_valid & POS_Z_VALID)!=0)
			{
				/* height control */
				HeightLoop(&fbvt, &wpt, &system_ctrler, 0.005);
				pos_loop_cnt = 0;
				fbvt.pos_valid &= (~POS_Z_VALID);
			}
		}
		
		/****************** velocity loop *********************/
		if(velo_loop_cnt++ >= VELO_LOOP_DIVIDER)
		{
			if((fbvt.velo_valid & VELO_X_VALID)!=0 && (fbvt.velo_valid & VELO_Y_VALID)!=0)
			{
				/* velocity control*/
				HorVeloLoop(&fbvt, &system_ctrler, 0.005);
				Pos2AngleMixer(system_ctrler.velo_x_ctrler.output
					, system_ctrler.velo_y_ctrler.output
					, &odt
					, fbvt.yaw_angle);
				velo_loop_cnt = 0;
				fbvt.velo_valid &= (~VELO_X_VALID);
				fbvt.velo_valid &= (~VELO_Y_VALID);
			}
			
			if((fbvt.velo_valid & VELO_Z_VALID)!=0)
			{
				HeightVeloLoop(&fbvt, &system_ctrler, 0.005);
				cpt.thrust_out = optional_param_global.miscel[0] + system_ctrler.velo_z_ctrler.output; 
				velo_loop_cnt = 0;
				fbvt.velo_valid &= (~VELO_Z_VALID);
			}
		}
		
		/****************** angle loop ************************/
		if(angle_loop_cnt++ >= ANGLE_LOOP_DIVIDER)
		{
			if((fbvt.angle_valid & ANGLE_ALL_VALID)!=0)
			{
				AngleLoop(&fbvt, &odt, &yaw_locked, &system_ctrler, 0.005);
			angle_loop_cnt = 0;
			}
			
		}
		
		/************************* rate loop ***********************/
		RateLoop(&fbvt, &system_ctrler, 0.005);
		
		cpt.roll_moment = system_ctrler.rollrate_ctrler.output;
		cpt.pitch_moment = system_ctrler.pitchrate_ctrler.output;
		cpt.yaw_moment = system_ctrler.yawrate_ctrler.output;//0.0;
		
		/************* decouple	 *************************/
		OutputControl(&cpt, &opt);
		
		/************* drive motor ***********************/
		if(odt.thrustOrder < 0.05)
		{
			opt.motor1_Out = 0.0;
			opt.motor2_Out = 0.0;
			opt.motor3_Out = 0.0;
			opt.motor4_Out = 0.0;
		}
		WriteMotor(&opt);

		/************ print message **********************/
		if(CNT++>=30)
		{
			CNT=0;
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f %.2f\r\n"
//									, odt.rollOrder
//									, odt.pitchOrder
//									, cpt.roll_moment
//									, cpt.pitch_moment);
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f %.2f\r\n"
//									, system_ctrler.px_ctrler.output
//									, system_ctrler.py_ctrler.output
//									, system_ctrler.velo_x_ctrler.output
//									, system_ctrler.velo_y_ctrler.output);
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f %.2f %.2f %.2f\r\n"
//									, fbvt.velo_x
//									, fbvt.velo_y
//									, system_ctrler.px_ctrler.output
//									, system_ctrler.py_ctrler.output
//									, system_ctrler.velo_x_ctrler.output
//									, system_ctrler.velo_y_ctrler.output);
//			sprintf(printf_buffer,"%d %d %d %d\r\n",opt.motor1_Out, opt.motor2_Out, opt.motor3_Out, opt.motor4_Out);
//			string_len = sprintf(printf_buffer, "%.2f %.2f\r\n", adt.pitchAngle*57.3, adt.pitchAngleRate*57.3);
//			string_len = sprintf(printf_buffer, "%.2f %.2f %.2f %.2f %.2f %.2f\n"
//						, fbvt.pos_x
//						, fbvt.pos_y
//						, fbvt.pos_z
//						, fbvt.velo_x
//						, fbvt.velo_y
//						, fbvt.velo_z);
//			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f %.2f %.2f %.2f\r\n"
//									, fbvt.roll_angle*57.3
//									, fbvt.pitch_angle*57.3
//									, fbvt.yaw_angle*57.3
//									, fbvt.roll_rate*57.3
//									, fbvt.pitch_rate*57.3
//									, fbvt.yaw_rate*57.3);
//			string_len = sprintf(printf_buffer, "%.2f %.2f %.2f\r\n", adt.rollAngle*57.3, adt.pitchAngle*57.3, adt.yawAngle*57.3);
			string_len = sprintf(printf_buffer,"%.2f %.2f %.2f %.2f\r\n"
									, opt.motor1_Out
									, opt.motor2_Out
									, opt.motor3_Out
									, opt.motor4_Out);
//			string_len = sprintf(printf_buffer,"%d %d %d %d\r\n"
//									, tim4IC1Width
//									, tim4IC2Width
//									, tim4IC3Width
//									, tim4IC4Width);
			UartSend(printf_buffer,string_len);
		}
		vTaskDelayUntil(&lastTime,(portTickType)(5/portTICK_RATE_MS));
	}
}

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
	}	
}

/*
note:
youmen 		capture from TIM4_3
pianhang 	capture from TIM4_4
fuyang		capture from TIM4_2
gunzhuan	capture from TIM4_1
*/
void InputControl(OrderType* odt)
{
	odt->thrustOrder = (tim4IC3Width-optional_param_global.RCneutral[2])*0.00125;	//کһۯ
	odt->yawOrder=(tim4IC4Width-optional_param_global.RCneutral[3])*0.003;	//֛̣ԉއ̙׈ָ®	 خճ30£/s
	odt->pitchOrder=(tim4IC2Width-optional_param_global.RCneutral[1])*0.0015;	//֛̣ԉއ׈ָ®	 خճ30£
	odt->rollOrder=(tim4IC1Width-optional_param_global.RCneutral[0])*0.0015;	//֛̣ԉއ׈ָ® خճ30£
	
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
	/*rate loop*/
	system_ctrler.rollrate_ctrler.kp = optional_param_global.loop_pid[0].xPID[0];
	system_ctrler.rollrate_ctrler.ki = optional_param_global.loop_pid[0].xPID[1];
	system_ctrler.rollrate_ctrler.kd = optional_param_global.loop_pid[0].xPID[2];
	system_ctrler.rollrate_ctrler.i_limit = 0.6;
	system_ctrler.rollrate_ctrler.d_limit = 10.0;
	system_ctrler.rollrate_ctrler.out_limit = 300.0;
	
	system_ctrler.pitchrate_ctrler.kp = optional_param_global.loop_pid[0].yPID[0];
	system_ctrler.pitchrate_ctrler.ki = optional_param_global.loop_pid[0].yPID[1];
	system_ctrler.pitchrate_ctrler.kd = optional_param_global.loop_pid[0].yPID[2];
	system_ctrler.pitchrate_ctrler.i_limit = 0.6;
	system_ctrler.pitchrate_ctrler.d_limit = 10.0;
	system_ctrler.pitchrate_ctrler.out_limit = 300.0;
	
	system_ctrler.yawrate_ctrler.kp = optional_param_global.loop_pid[0].zPID[0];
	system_ctrler.yawrate_ctrler.ki = optional_param_global.loop_pid[0].zPID[1];
	system_ctrler.yawrate_ctrler.kd = optional_param_global.loop_pid[0].zPID[2];
	system_ctrler.yawrate_ctrler.i_limit = 0.6;
	system_ctrler.yawrate_ctrler.d_limit = 10.0;
	system_ctrler.yawrate_ctrler.out_limit = 300.0;
	
	/*angle loop*/
	system_ctrler.roll_ctrler.kp = optional_param_global.loop_pid[1].xPID[0];
	system_ctrler.roll_ctrler.ki = optional_param_global.loop_pid[1].xPID[1];
	system_ctrler.roll_ctrler.kd = optional_param_global.loop_pid[1].xPID[2];
	system_ctrler.roll_ctrler.i_limit = 0.6;
	system_ctrler.roll_ctrler.d_limit = 1.0;
	system_ctrler.roll_ctrler.out_limit = 4.0;
	
	system_ctrler.pitch_ctrler.kp = optional_param_global.loop_pid[1].yPID[0];
	system_ctrler.pitch_ctrler.ki = optional_param_global.loop_pid[1].yPID[1];
	system_ctrler.pitch_ctrler.kd = optional_param_global.loop_pid[1].yPID[2];
	system_ctrler.pitch_ctrler.i_limit = 0.6;
	system_ctrler.pitch_ctrler.d_limit = 1.0;
	system_ctrler.pitch_ctrler.out_limit = 4.0;
	
	system_ctrler.yaw_ctrler.kp = optional_param_global.loop_pid[1].zPID[0];
	system_ctrler.yaw_ctrler.ki = optional_param_global.loop_pid[1].zPID[1];
	system_ctrler.yaw_ctrler.kd = optional_param_global.loop_pid[1].zPID[2];
	system_ctrler.yaw_ctrler.i_limit = 0.6;
	system_ctrler.yaw_ctrler.d_limit = 1.0;
	system_ctrler.yaw_ctrler.out_limit = 4.0;
	
	/*velo loop*/
	system_ctrler.velo_x_ctrler.kp = optional_param_global.loop_pid[2].xPID[0];
	system_ctrler.velo_x_ctrler.ki = optional_param_global.loop_pid[2].xPID[1];
	system_ctrler.velo_x_ctrler.kd = optional_param_global.loop_pid[2].xPID[2];
	system_ctrler.velo_x_ctrler.i_limit = 1.0;
	system_ctrler.velo_x_ctrler.d_limit = 5.0;
	system_ctrler.velo_x_ctrler.out_limit = 0.6;
	
	system_ctrler.velo_y_ctrler.kp = optional_param_global.loop_pid[2].yPID[0];
	system_ctrler.velo_y_ctrler.ki = optional_param_global.loop_pid[2].yPID[1];
	system_ctrler.velo_y_ctrler.kd = optional_param_global.loop_pid[2].yPID[2];
	system_ctrler.velo_y_ctrler.i_limit = 1.0;
	system_ctrler.velo_y_ctrler.d_limit = 5.0;
	system_ctrler.velo_y_ctrler.out_limit = 0.6;
	
	system_ctrler.velo_z_ctrler.kp = optional_param_global.loop_pid[2].zPID[0];
	system_ctrler.velo_z_ctrler.ki = optional_param_global.loop_pid[2].zPID[1];
	system_ctrler.velo_z_ctrler.kd = optional_param_global.loop_pid[2].zPID[2];
	system_ctrler.velo_z_ctrler.i_limit = 1.0;
	system_ctrler.velo_z_ctrler.d_limit = 5.0;
	system_ctrler.velo_z_ctrler.out_limit = 0.6;
	
	/*pos loop*/
	system_ctrler.px_ctrler.kp = optional_param_global.loop_pid[3].xPID[0];
	system_ctrler.px_ctrler.ki = optional_param_global.loop_pid[3].xPID[1];
	system_ctrler.px_ctrler.kd = optional_param_global.loop_pid[3].xPID[2];
	system_ctrler.px_ctrler.i_limit = 1.0;
	system_ctrler.px_ctrler.d_limit = 4.0;
	system_ctrler.px_ctrler.out_limit = 0.5;
	
	system_ctrler.py_ctrler.kp = optional_param_global.loop_pid[3].yPID[0];
	system_ctrler.py_ctrler.ki = optional_param_global.loop_pid[3].yPID[1];
	system_ctrler.py_ctrler.kd = optional_param_global.loop_pid[3].yPID[2];
	system_ctrler.py_ctrler.i_limit = 1.0;
	system_ctrler.py_ctrler.d_limit = 4.0;
	system_ctrler.py_ctrler.out_limit = 0.5;
	
	system_ctrler.height_ctrler.kp = optional_param_global.loop_pid[3].zPID[0];
	system_ctrler.height_ctrler.ki = optional_param_global.loop_pid[3].zPID[1];
	system_ctrler.height_ctrler.kd = optional_param_global.loop_pid[3].zPID[2];
	system_ctrler.height_ctrler.i_limit = 1.0;
	system_ctrler.height_ctrler.d_limit = 4.0;
	system_ctrler.height_ctrler.output = 0.5;
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
				system_ctrler.velo_x_ctrler.kp = optional_param_global.loop_pid[2].xPID[0];
				system_ctrler.velo_x_ctrler.ki = optional_param_global.loop_pid[2].xPID[1];
				system_ctrler.velo_x_ctrler.kd = optional_param_global.loop_pid[2].xPID[2];
				
				system_ctrler.velo_y_ctrler.kp = optional_param_global.loop_pid[2].yPID[0];
				system_ctrler.velo_y_ctrler.ki = optional_param_global.loop_pid[2].yPID[1];
				system_ctrler.velo_y_ctrler.kd = optional_param_global.loop_pid[2].yPID[2];
				
				system_ctrler.velo_z_ctrler.kp = optional_param_global.loop_pid[2].zPID[0];
				system_ctrler.velo_z_ctrler.ki = optional_param_global.loop_pid[2].zPID[1];
				system_ctrler.velo_z_ctrler.kd = optional_param_global.loop_pid[2].zPID[2];
			break;
		case 3:
			system_ctrler.px_ctrler.kp = optional_param_global.loop_pid[3].xPID[0];
			system_ctrler.px_ctrler.ki = optional_param_global.loop_pid[3].xPID[1];
			system_ctrler.px_ctrler.kd = optional_param_global.loop_pid[3].xPID[2];
			
			system_ctrler.py_ctrler.kp = optional_param_global.loop_pid[3].yPID[0];
			system_ctrler.py_ctrler.ki = optional_param_global.loop_pid[3].yPID[1];
			system_ctrler.py_ctrler.kd = optional_param_global.loop_pid[3].yPID[2];
			
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
	
	ctrler->err = ctrler->desired - ctrler->actual;
	if(info->err_filter != NULL)
		ctrler->err = GaussianFilter((GFilterType *)(info->err_filter), ctrler->err);
	
	if(info->use_ref_diff == 0)
		ctrler->deriv = (ctrler->err - ctrler->prev_err)/info->dt;
	
	if(info->deriv_filter != NULL)
		ctrler->deriv = GaussianFilter((GFilterType *)(info->deriv_filter), ctrler->deriv);
	
	ctrler->prev_err = ctrler->err;
	ctrler->integ += ctrler->err*info->dt;
	
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
	
	if(ctrler->output > ctrler->out_limit)
		ctrler->output = ctrler->out_limit;
	else if(ctrler->output < -ctrler->out_limit)
		ctrler->output = -ctrler->out_limit;
}

void Pos2AngleMixer(float xPID, float yPID, OrderType *odt, float yawAngle)
{
	float cosfi = arm_cos_f32(yawAngle);
	float sinfi = arm_sin_f32(yawAngle);	
	/*decouple*/
	odt->pitchOrder = -(xPID*cosfi+yPID*sinfi);
	odt->rollOrder = -xPID*sinfi+yPID*cosfi;
}

void OutputControl(CtrlProcType *cpt, OutputType* opt)
{
	/*restrict output thrust to guarrentee angle PID allowance*/
	if(cpt->thrust_out>0.0 && cpt->thrust_out < 0.17)
	{
		opt->motor1_Out = cpt->thrust_out;
		opt->motor2_Out = cpt->thrust_out;
		opt->motor3_Out = cpt->thrust_out;
		opt->motor4_Out = cpt->thrust_out;
	}
	else if(cpt->thrust_out >= 0.17 && cpt->thrust_out < 0.80)
	{
		opt->motor1_Out = cpt->thrust_out + cpt->roll_moment + cpt->pitch_moment - cpt->yaw_moment;
		opt->motor2_Out = cpt->thrust_out + cpt->roll_moment - cpt->pitch_moment + cpt->yaw_moment;
		opt->motor3_Out = cpt->thrust_out - cpt->roll_moment - cpt->pitch_moment - cpt->yaw_moment;
		opt->motor4_Out = cpt->thrust_out - cpt->roll_moment + cpt->pitch_moment + cpt->yaw_moment;
	}
	else if(cpt->thrust_out >= 0.80)
	{
		opt->motor1_Out = 0.80 + cpt->roll_moment + cpt->pitch_moment - cpt->yaw_moment;
		opt->motor2_Out = 0.80 + cpt->roll_moment - cpt->pitch_moment + cpt->yaw_moment;
		opt->motor3_Out = 0.80 - cpt->roll_moment - cpt->pitch_moment - cpt->yaw_moment;
		opt->motor4_Out = 0.80 - cpt->roll_moment + cpt->pitch_moment + cpt->yaw_moment;
	}
	
//	opt->motor1_Out = youmenOut + cpt->pitch_moment - cpt->yaw_moment;
//	opt->motor2_Out = youmenOut + cpt->roll_moment  + cpt->yaw_moment;
//	opt->motor3_Out = youmenOut - cpt->pitch_moment - cpt->yaw_moment;
//	opt->motor4_Out = youmenOut - cpt->roll_moment + cpt->yaw_moment;

	if(opt->motor1_Out<0.1) 
		opt->motor1_Out=0.1;
	else if(opt->motor1_Out>0.95) 
		opt->motor1_Out=0.95; 
		
	if(opt->motor2_Out<0.1) 
		opt->motor2_Out=0.1;
	else if(opt->motor2_Out>0.95) 
		opt->motor2_Out=0.95; 
		
	if(opt->motor3_Out<0.1) 
		opt->motor3_Out=0.1;
	else if(opt->motor3_Out>0.95) 
		opt->motor3_Out=0.95; 
		
	if(opt->motor4_Out<0.1) 
		opt->motor4_Out=0.1;
	else if(opt->motor4_Out>0.95) 
		opt->motor4_Out=0.95; 
	
	if(cpt->thrust_out<0.05)
	{
		opt->motor1_Out = 0.0;
		opt->motor2_Out = 0.0;
		opt->motor3_Out = 0.0;
		opt->motor4_Out = 0.0;
	}
}

/* this function are platform relevant*/
void WriteMotor(OutputType* opt)
{
	if(opt->motor1_Out < 0.01)
		TIM_SetCompare1(TIM3, 100);	//youmenOut 	 
	else
		TIM_SetCompare1(TIM3, (u16)(200+opt->motor1_Out*1000));
		
	if(opt->motor2_Out < 0.01)
		TIM_SetCompare2(TIM3, 100);	//youmenOut 	 
	else
		TIM_SetCompare2(TIM3, (u16)(200+opt->motor2_Out*1000));
		
	if(opt->motor3_Out < 0.01)
		TIM_SetCompare3(TIM3, 100);	//youmenOut 	 
	else
		TIM_SetCompare3(TIM3, (u16)(200+opt->motor3_Out*1000));
		
	if(opt->motor4_Out < 0.01)
		TIM_SetCompare4(TIM3, 100);	//youmenOut 	 
	else
		TIM_SetCompare4(TIM3, (u16)(200+opt->motor4_Out*1000));

//	TIM_SetCompare1(TIM3,100);	//youmenOut 	 
//	TIM_SetCompare2(TIM3,100);	//youmenOut 	  
//	TIM_SetCompare3(TIM3,100);	//youmenOut	  
//	TIM_SetCompare4(TIM3,100);	//youmenOut	
}	

/* roll the roll-stick rightmost to leftmost to start*/
void WaitRCSignal(void)
{
	u16 i=0;
	float neutral[4] = {0.0};

	while(tim4IC1Width<1800)
	{
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}
	while(tim4IC1Width>1800)
	{
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}
	while(tim4IC1Width>1200)
	{
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}
	while(tim4IC1Width<1200)
	{
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}

	for(i=0; i<50; i++)
	{
		neutral[0] += tim4IC1Width;
		neutral[1] += tim4IC2Width;
		neutral[2] += tim4IC3Width;
		neutral[3] += tim4IC4Width;
		vTaskDelay((portTickType)(20/portTICK_RATE_MS));
	}

	optional_param_global.RCneutral[0] = (s16)(neutral[0]/50);
	optional_param_global.RCneutral[1] = (s16)(neutral[1]/50);
	optional_param_global.RCneutral[2] = (s16)(neutral[2]/50);
	optional_param_global.RCneutral[3] = (s16)(neutral[3]/50);
}

void FeedBack(FeedBackValType *fbvt, AHRSDataType *adt, PosDataType *pdt, portBASE_TYPE pos_data_valid)
{
	if(pos_data_valid == pdPASS)
	{
		fbvt->pos_x = pdt->posX;
		fbvt->pos_y = pdt->posY;
		fbvt->pos_z = pdt->posZ;
		fbvt->pos_valid = POS_Z_VALID | POS_X_VALID | POS_Y_VALID;
		
		fbvt->velo_x = pdt->veloX;
		fbvt->velo_y = pdt->veloY;
		fbvt->velo_z = pdt->veloZ;
		fbvt->velo_valid = POS_Z_VALID | POS_X_VALID | POS_Y_VALID;
		
		Blinks(LED1,1);
	}
	
	fbvt->roll_angle = adt->rollAngle;
	fbvt->pitch_angle = adt->pitchAngle;
	fbvt->yaw_angle = adt->yawAngle;
	fbvt->angle_valid = ROLL_ANGLE_VALID | PITCH_ANGLE_VALID | YAW_ANGLE_VALID;
//		fbvt->angle_valid = 0;
	
	fbvt->roll_rate = adt->rollAngleRate;
	fbvt->pitch_rate = adt->pitchAngleRate;
	fbvt->yaw_rate = adt->yawAngleRate;
	fbvt->rate_valid = ROLL_RATE_VALID | PITCH_RATE_VALID | YAW_RATE_VALID;	
}

void PosLoop(FeedBackValType *fbvt, WayPointType *wpt, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	msg2ctrler.in = wpt->x*0.001;
	msg2ctrler.fb = fbvt->pos_x;
	msg2ctrler.dt = dt * POS_LOOP_DIVIDER;
	msg2ctrler.use_ref_diff = 0;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.err_filter = NULL;

	PIDProccessing(&(system_ctrler->px_ctrler), &msg2ctrler);

	msg2ctrler.in = wpt->y*0.001;
	msg2ctrler.fb = fbvt->pos_y;
	msg2ctrler.dt = dt * POS_LOOP_DIVIDER;
	msg2ctrler.use_ref_diff = 0;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.err_filter = NULL;

	PIDProccessing(&(system_ctrler->py_ctrler), &msg2ctrler);
}

void HorVeloLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	msg2ctrler.in = system_ctrler->px_ctrler.output;
	msg2ctrler.fb = fbvt->velo_x;
	msg2ctrler.dt = dt * VELO_LOOP_DIVIDER;
	msg2ctrler.use_ref_diff = 0;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.err_filter = NULL;

	PIDProccessing(&(system_ctrler->velo_x_ctrler), &msg2ctrler);

	msg2ctrler.in = system_ctrler->py_ctrler.output;
	msg2ctrler.fb = fbvt->velo_y;
	msg2ctrler.dt = dt * VELO_LOOP_DIVIDER;
	msg2ctrler.use_ref_diff = 0;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.err_filter = NULL;

	PIDProccessing(&(system_ctrler->velo_y_ctrler), &msg2ctrler);	
}

void HeightLoop(FeedBackValType *fbvt, WayPointType *wpt, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	msg2ctrler.in = wpt->height*0.001;
	msg2ctrler.fb = fbvt->pos_z;
	msg2ctrler.dt = dt * POS_LOOP_DIVIDER;
	msg2ctrler.use_ref_diff = 0;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.err_filter = NULL;
	
	PIDProccessing(&(system_ctrler->height_ctrler), &msg2ctrler);	
}

void HeightVeloLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	msg2ctrler.in = system_ctrler->height_ctrler.output;
	msg2ctrler.fb = -fbvt->velo_z;
	msg2ctrler.dt = dt * VELO_LOOP_DIVIDER;
	msg2ctrler.err_filter = (void *)&dheight_lpf;
	msg2ctrler.deriv_filter = NULL;
	msg2ctrler.use_ref_diff = 0;
	
	PIDProccessing(&(system_ctrler->velo_z_ctrler), &msg2ctrler);	
}

void AngleLoop(FeedBackValType *fbvt, OrderType *odt, float *desired_yaw, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	/******************* roll*******************/
	if((fbvt->angle_valid & ROLL_ANGLE_VALID) != 0)
	{
		msg2ctrler.in = odt->rollOrder;
		msg2ctrler.fb = fbvt->roll_angle;
		msg2ctrler.dt = dt * ANGLE_LOOP_DIVIDER;
		msg2ctrler.use_ref_diff = 0;
		msg2ctrler.deriv_filter = NULL;
		msg2ctrler.err_filter = NULL;
		
		PIDProccessing(&(system_ctrler->roll_ctrler),&msg2ctrler);
	}
	else
	{
		ResetCtrler(&system_ctrler->roll_ctrler);
	}
	
	/******************* pitch *******************/
	if((fbvt->angle_valid & PITCH_ANGLE_VALID) != 0)
	{
		msg2ctrler.in = odt->pitchOrder;
		msg2ctrler.fb = fbvt->pitch_angle;
		msg2ctrler.dt = dt * ANGLE_LOOP_DIVIDER;
		msg2ctrler.deriv_filter = NULL;
		msg2ctrler.err_filter = NULL;
		msg2ctrler.use_ref_diff = 0;
		
		PIDProccessing(&(system_ctrler->pitch_ctrler), &msg2ctrler);
	}
	else
	{
		ResetCtrler(&system_ctrler->pitch_ctrler);
	}
	/*yaw loop*/
	if((fbvt->angle_valid & YAW_ANGLE_VALID) != 0)
	{
		if(odt->yawOrder<0.05 && odt->yawOrder>-0.05)
		{
			if(*desired_yaw - fbvt->yaw_angle > PI)
				fbvt->yaw_angle += 2*PI;
			else if(*desired_yaw - fbvt->yaw_angle < -PI)
				fbvt->yaw_angle -= 2*PI;
			
			msg2ctrler.in = *desired_yaw;
			msg2ctrler.fb = fbvt->yaw_angle;
			msg2ctrler.dt = dt * ANGLE_LOOP_DIVIDER;
			msg2ctrler.deriv_filter = NULL;
			msg2ctrler.err_filter = NULL;
			msg2ctrler.use_ref_diff = 0;
			
			PIDProccessing(&(system_ctrler->yaw_ctrler), &msg2ctrler);
		}
		else
		{
			/*no need to reset yaw ctrler, the state of 
			which is exactly the output to hold yawrate to zero*/
			*desired_yaw = fbvt->yaw_angle;
			system_ctrler->yaw_ctrler.output = odt->yawOrder
											+ system_ctrler->yaw_ctrler.kp * system_ctrler->yaw_ctrler.err
											+ system_ctrler->yaw_ctrler.ki * system_ctrler->yaw_ctrler.integ
											+ system_ctrler->yaw_ctrler.kd * system_ctrler->yaw_ctrler.deriv;
		}	
	}
	else
	{
		ResetCtrler(&system_ctrler->yaw_ctrler);
		system_ctrler->yaw_ctrler.output = odt->yawOrder;
	}
}

void RateLoop(FeedBackValType *fbvt, struct system_level_ctrler *system_ctrler, float dt)
{
	PIDCtrlerAuxiliaryType msg2ctrler;
	/********** roll ***********/
	if((fbvt->rate_valid & ROLL_RATE_VALID) != 0)
	{
		msg2ctrler.in = system_ctrler->roll_ctrler.output;
		msg2ctrler.fb = fbvt->roll_rate;
		msg2ctrler.dt = dt;
		msg2ctrler.deriv_filter = (void *)&droll_rate_lpf;
		msg2ctrler.err_filter = (void *)&roll_rate_lpf;
		msg2ctrler.use_ref_diff = 0;
		
		PIDProccessing(&(system_ctrler->rollrate_ctrler), &msg2ctrler);
	}
	else
	{
		ResetCtrler(&system_ctrler->rollrate_ctrler);
	}
	/********** pitch ***********/
	if((fbvt->rate_valid & PITCH_RATE_VALID) != 0)
	{
		msg2ctrler.in = system_ctrler->pitch_ctrler.output;
		msg2ctrler.fb = fbvt->pitch_rate;
		msg2ctrler.dt = dt;
		msg2ctrler.deriv_filter = (void *)&dpitch_rate_lpf;
		msg2ctrler.err_filter = (void *)&pitch_rate_lpf;
		msg2ctrler.use_ref_diff = 0;
		
		PIDProccessing(&(system_ctrler->pitchrate_ctrler), &msg2ctrler);
	}
	else
	{
		ResetCtrler(&system_ctrler->pitchrate_ctrler);
	}
	/********** yaw ***********/
	if((fbvt->rate_valid & YAW_RATE_VALID) != 0)
	{	
		msg2ctrler.in = system_ctrler->yaw_ctrler.output;
		msg2ctrler.fb = fbvt->yaw_rate;
		msg2ctrler.dt = dt;
		msg2ctrler.deriv_filter = (void *)&dyaw_rate_lpf;
		msg2ctrler.err_filter = (void *)&yaw_rate_lpf;
		msg2ctrler.use_ref_diff = 0;
		PIDProccessing(&(system_ctrler->yawrate_ctrler), &msg2ctrler);
	}
	else
	{
		ResetCtrler(&system_ctrler->yawrate_ctrler);
	}	
}
