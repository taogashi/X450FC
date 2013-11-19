#include "flightConTask.h"
#include "AHRSEKF.h"
#include "INSEKF.h"
//#include "sensor.h"
#include "pwm.h"
#include "diskTask.h"
#include "uartTask.h"
#include "ledTask.h"
#include <arm_math.h>
#include <stdio.h>

const char* PARA_FORMAT_IN="PARA,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%hd,%hd,%hd,%hd,%hd";
const char* PARA_FORMAT_OUT="PARA,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d\r\n";
const char* PID_FORMAT_IN="PID,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f";
const char* PID_FORMAT_OUT="set PID para: %.2f,%.2f,%.2f\r\n%.2f,%.2f,%.2f,%.2f\r\n%.2f,%.2f,%.2f\r\n%.2f,%.2f,%.2f\r\n";
const char* NEUTRAL_FORMAT_IN="Neutral,%f,%hd,%hd,%hd,%hd";
const char* NEUTRAL_FORMAT_OUT="set Neutral para: %.2f\r\n%d,%d,%d,%d\r\n";
const char* WAYPOINT_FORMAT_IN="Waypoint,%hd,%hd,%hd,%hd,%d,%d,%d,%d";
const char* WAYPOINT_FORMAT_OUT="Waypoint,%d,%d,%d,%d,%d,%d,%d,%d\r\n";

s16 CalChecksum(OptionalPara* OP)
{
	s16 sum=0;
	sum = (s16)(OP->rollP)
		+(s16)(OP->rollD)
		+(s16)(OP->rollI)
		+(s16)(OP->yawP)
		+(s16)(OP->yawD1)
		+(s16)(OP->yawD2)
		+(s16)(OP->yawI)
		+(s16)(OP->horiP)
		+(s16)(OP->horiD)
		+(s16)(OP->horiI)
		+(s16)(OP->heightP)
		+(s16)(OP->heightD)
		+(s16)(OP->heightI)
		+(s16)(OP->hoverThrust)
		+OP->RCneutral[0]
		+OP->RCneutral[1]
		+OP->RCneutral[2]
		+OP->RCneutral[3];
	return sum;
}

//计算偏航控制的D参数
float CalYawD(float yawRate1,float yawD1,float yawRate2,float yawD2,float newYawRate)
{
	float tYawD;
	float absYawRate=fabs(newYawRate);
	float k=(yawD2-yawD1)/(yawRate2-yawRate1);//斜率，是个负值
	if(absYawRate<yawRate1)
		tYawD=yawD1;

	else if(absYawRate<yawRate2)
	{
		if(newYawRate>0)
			tYawD=yawD1+(newYawRate-yawRate1)*k;
		else
			tYawD=yawD1+(newYawRate+yawRate1)*(-k);
	}
	else 
		tYawD=yawD2;
	return tYawD;
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
void InputControl(OrderType* odt,OptionalPara* OP)
{
	odt->thrustOrder = (tim4IC3Width-OP->RCneutral[2])*0.00125;	//归一化
	odt->yawOrder=(tim4IC4Width-OP->RCneutral[3])*0.003;	//折算成角速度指令	 最大30°/s

	odt->pitchOrder=(tim4IC2Width-OP->RCneutral[1])*0.002;	//折算成角度指令	 最大30°
	odt->rollOrder=(tim4IC1Width-OP->RCneutral[0])*0.002;	//折算成角度指令 最大30°
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
void AttControl(AttConDataType *acdt,AttConValType* acvt,OrderType* odt,OptionalPara* OP)
{
	/*有人操作情况下的PID*/
	//遥控器输入命令为偏航角速度、滚转角、俯仰角。
	//三者的控制为有静差控制（需要有人操作）。没有偏航指令时PI控制，保持方向、偏航角无静差
	//偏航用P调节
	float yawD;	
	float roll_e1=0.0;	
	float nick_e1=0.0;

	static float rollErrInteg=0.0;
	static float nickErrInteg=0.0;

	float yaw_err=0.0;
	static float yawErrInteg=0.0;
	static float curHeading=0.0;	   //当前的机头指向

//-------------------------------------------roll---------------------------------------------													
	roll_e1 = acdt->rollAngle - odt->rollOrder;		//误差				
	rollErrInteg+= roll_e1;
	if(acvt->IntegerEnableOption==0)			   //
	{
		rollErrInteg=0;
	}
	else if(rollErrInteg<-80.0)
	{
		rollErrInteg=-80.0;
	}
	else if(rollErrInteg>80.0)
	{
		rollErrInteg=80.0;
	}

	acvt->rollConVal = OP->rollP*(roll_e1) + OP->rollD*(acdt->rollAngleRate) + OP->rollI*rollErrInteg;			// PID调节器	rollstate量纲是角度

//-----------------------------------------------nick------------------------------------
	nick_e1 = acdt->pitchAngle-odt->pitchOrder;
	nickErrInteg+= nick_e1;
	if(acvt->IntegerEnableOption==0)
	{
		nickErrInteg=0;
	}
	else if(nickErrInteg<-80.0)
	{
		nickErrInteg=-80.0;
	}
	else if(nickErrInteg>80.0)
	{
		nickErrInteg=80.0;
	}
	acvt->pitchConVal = OP->rollP*(nick_e1) + OP->rollD*(acdt->pitchAngleRate)+ OP->rollI*nickErrInteg;	 //PD调节器//	

//---------------------------------------------------------yaw----------------------------------------------
	if(odt->yawOrder>-0.03 && odt->yawOrder<0.03)	//此时认为遥控器无输入
	{
		yawD=CalYawD(0.1,OP->yawD1,0.8,OP->yawD2,acdt->yawAngleRate);

		yaw_err=acdt->yawAngle-curHeading;
		if(yaw_err > 3.1415926)
			yaw_err = yaw_err - 3.1415926*2.0;
		else if(yaw_err < -3.1415926)
			yaw_err = yaw_err + 3.1415926*2.0;
		
		yawErrInteg+= yaw_err;
		if(acvt->IntegerEnableOption==0)			   //
		{
			yawErrInteg=0;
		}
		else if(yawErrInteg<-100.0)
		{
			yawErrInteg=-100.0;
		}
		else if(yawErrInteg>100.0)
		{
			yawErrInteg=100.0;
		}
		acvt->yawConVal=OP->yawP*yaw_err+yawD*acdt->yawAngleRate+OP->yawI*yawErrInteg;	   //	   
	}
	else
	{	
		curHeading=acdt->yawAngle;
		yaw_err=acdt->yawAngleRate-odt->yawOrder;
		acvt->yawConVal=(OP->yawD1+OP->yawD2)*1.2*yaw_err;
	}
}

void PosControl(OrderType* odt,PosConDataType* pcdt,AttConDataType *acdt,OptionalPara* OP)//
{
	static float targetXPos=0.0;
	static float targetYPos=0.0;
	float xPosErr=0.0;
	float yPosErr=0.0;
	static float xPosErrIntg=0.0;
	static float yPosErrIntg=0.0;

	float tarHeight=odt->thrustOrder*20.0;
	float heightErr=0.0;
	static float heightErrInteg=0.0;

	float xPID,yPID;
	
	char printf_buffer[100];
	//高度控制
//	if(odt->thrustOrder<0.05) odt->thrustOut=0;
//	else
//	{
//		//高度误差
//		heightErr = pcdt->posZ-tarHeight;
//		//高度误差积分
//		heightErrInteg+=heightErr*0.005; 
//		//积分限幅
//		if(heightErrInteg>1.0) heightErrInteg=1.0;
//		if(heightErrInteg<-1.0) heightErrInteg=-1.0;
//	
//		odt->thrustOut=OP->hoverThrust - OP->heightP*heightErr + OP->heightD*pcdt->veloZ - OP->heightI*heightErrInteg;
//	}
//
//	if(odt->thrustOut>1) odt->thrustOut=1;
//	else if(odt->thrustOut<0) odt->thrustOut=0;

//	if(pcdt->veloZ<0.03 && pcdt->veloZ>-0.03 && odt->thrustOut>0.4) OP->hoverThrust=0.99*OP->hoverThrust+0.01*odt->thrustOut;

	//水平位置控制
	if(tim5IC1Width>1500)
	{
		targetXPos=pcdt->posX;
		targetYPos=pcdt->posY;
		xPosErrIntg = 0.0;
		yPosErrIntg = 0.0;
	}
	else
	{
		if((odt->rollOrder<0.02 && odt->rollOrder>-0.02) && (odt->pitchOrder<0.02 && odt->pitchOrder>-0.02))
		{
			float cosfi = arm_cos_f32(acdt->yawAngle);
			float sinfi = arm_sin_f32(acdt->yawAngle);	
			xPosErr=pcdt->posX-targetXPos;
			yPosErr=pcdt->posY-targetYPos;
			xPosErrIntg += xPosErr*0.01;
			if(xPosErrIntg > 50.0) xPosErrIntg=50.0;
			else if(xPosErrIntg <-50.0) xPosErrIntg=-50.0;
			yPosErrIntg += yPosErr;
			if(yPosErrIntg > 50.0) yPosErrIntg=50.0;
			else if(yPosErrIntg <-50.0) yPosErrIntg=-50.0;
			
			/*PID*/
			xPID = OP->horiP*xPosErr + OP->horiD*pcdt->veloX + OP->horiI*xPosErrIntg;
			yPID = OP->horiP*yPosErr + OP->horiD*pcdt->veloY + OP->horiI*yPosErrIntg;
			
			/*decouple*/
			odt->pitchOrder = 0.03*(xPID*cosfi+yPID*sinfi);
			if(odt->pitchOrder > 0.18) odt->pitchOrder=0.18;
			else if(odt->pitchOrder < -0.18) odt->pitchOrder=-0.18;

			odt->rollOrder = -0.03*(-xPID*sinfi+yPID*cosfi);
			if(odt->rollOrder > 0.18) odt->rollOrder=0.18;
			else if(odt->rollOrder < -0.18) odt->rollOrder=-0.18;
			
			sprintf(printf_buffer,"%.2f %.2f\r\n",xPosErr,yPosErr);
			Uart2Send(printf_buffer,strlen(printf_buffer));
		}	
		else
		{
			targetXPos=pcdt->posX;
			targetYPos=pcdt->posX;
			xPosErrIntg=0.0;
			yPosErrIntg=0.0;
		}	
	}
	odt->thrustOut=odt->thrustOrder;
}

/* 作用：根据acvt指令和姿态控制量，计算输出给四个电机的转速指令
 * 参数：
 * 	odt
 * 	opt
 * 返回值：无*/
void OutputControl(AttConValType* acvt,OrderType* odt,OutputType* opt)
{
	s16 youmenOut=200+(s16)(odt->thrustOut*800);
	if(youmenOut<250) youmenOut=100;

	opt->motor1_Out=youmenOut+acvt->yawConVal-acvt->rollConVal-acvt->pitchConVal;
	opt->motor2_Out=youmenOut-acvt->yawConVal-acvt->rollConVal+acvt->pitchConVal;
	opt->motor3_Out=youmenOut+acvt->yawConVal+acvt->rollConVal+acvt->pitchConVal;
	opt->motor4_Out=youmenOut-acvt->yawConVal+acvt->rollConVal-acvt->pitchConVal;

//	opt->motor1_Out=youmenOut+acvt->yawConVal-acvt->pitchConVal;
//	opt->motor2_Out=youmenOut-acvt->yawConVal-acvt->rollConVal;
//	opt->motor3_Out=youmenOut+acvt->yawConVal+acvt->pitchConVal;
//	opt->motor4_Out=youmenOut-acvt->yawConVal+acvt->rollConVal;

	if(opt->motor1_Out<250) opt->motor1_Out=100;
	if(opt->motor1_Out>1100) opt->motor1_Out=1100; 

	if(opt->motor2_Out<250) opt->motor2_Out=100;
	if(opt->motor2_Out>1100) opt->motor2_Out=1100;

	if(opt->motor3_Out<250) opt->motor3_Out=100;
	if(opt->motor3_Out>1100) opt->motor3_Out=1100;

	if(opt->motor4_Out<250) opt->motor4_Out=100;
	if(opt->motor4_Out>1100) opt->motor4_Out=1100;
	
	if(odt->thrustOrder<0.05)
	{
		opt->motor1_Out=100;
		opt->motor2_Out=100;
		opt->motor3_Out=100;
		opt->motor4_Out=100;
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
	char printf_buffer[100];
	
	portBASE_TYPE xdisk_read_status;
	portBASE_TYPE xstatus;
	u8 CNT=0;
	u8 posConCNT=0;
	PosConDataType pcdt;
	
	//acdt contains attitude angle and angle rate
	AttConDataType acdt;
	
	//attitude PID output
	AttConValType acvt={0,0,0,0};
	
	//orders from remote controller
	OrderType odt;
	
	//final thrust output to each motor
	OutputType opt={100,100,100,100};

	//
	OptionalPara OP;
	OptionalPara tempOP;

	portTickType lastTime;

	//at this stage, flightConTask wait parameters read form disk
	sprintf(printf_buffer, "load para...\r\n");
	Uart2Send(printf_buffer, strlen(printf_buffer));
	xdisk_read_status = xQueueReceive(xDiskWrQueue1,&OP,(portTickType)(5000/portTICK_RATE_MS));
	if(xdisk_read_status == pdPASS)
	{
		sprintf(printf_buffer, "OK!\r\n");
		Uart2Send(printf_buffer, 5);
		sprintf(printf_buffer,PID_FORMAT_OUT
					,OP.rollP,OP.rollD,OP.rollI
					,OP.yawP,OP.yawD1,OP.yawD2,OP.yawI
					,OP.horiP,OP.horiD,OP.horiI
					,OP.heightP,OP.heightD,OP.heightI);
		Uart2Send(printf_buffer, strlen(printf_buffer));
		Blinks(LED1,1);
	}
	//if fails, read from uart
	else
	{
		sprintf(printf_buffer,"failed to read parameters!\r\n");
		Uart2Send(printf_buffer, strlen(printf_buffer));
		while(xdisk_read_status != pdPASS)
		{
			xstatus = xQueueReceive(xUartParaQueue,&tempOP,0);
			if(xstatus == pdPASS)
			{
				if(tempOP.checksum == 0)//PID para
				{
					OP.rollP=tempOP.rollP;
					OP.rollD=tempOP.rollD;
					OP.rollI=tempOP.rollI;
					
					OP.yawP=tempOP.yawP;
					OP.yawD1=tempOP.yawD1;
					OP.yawD2=tempOP.yawD2;
					OP.yawI=tempOP.yawI;

					OP.horiP=tempOP.horiP;
					OP.horiD=tempOP.horiD;
					OP.horiI=tempOP.horiI;

					OP.heightP=tempOP.heightP;
					OP.heightD=tempOP.heightD;
					OP.heightI=tempOP.heightI;
					
					xdisk_read_status = pdPASS;
					Blinks(LED1,1);
				}
				else if(tempOP.checksum==1)//Neutral
				{
					OP.hoverThrust = tempOP.hoverThrust;
					OP.RCneutral[0] = tempOP.RCneutral[0];
					OP.RCneutral[1] = tempOP.RCneutral[1];
					OP.RCneutral[2] = tempOP.RCneutral[2];
					OP.RCneutral[3] = tempOP.RCneutral[3];
				}
				OP.checksum = CalChecksum(&OP);
				xQueueSend(xDiskWrQueue1,&OP,0);//write to SD card
			}
			vTaskDelay((portTickType)(20/portTICK_RATE_MS));
		}
	}
		
	TIM4_IT_Config();
	TIM5_IT_Config();
	
	lastTime = xTaskGetTickCount();

	for(;;)
	{
		/*handle uart parameters receiving event*/
		xstatus = xQueueReceive(xUartParaQueue,&tempOP,0);
		if(xstatus == pdPASS)
		{
			if(tempOP.checksum == 0)//PID para
			{
				OP.rollP=tempOP.rollP;
				OP.rollD=tempOP.rollD;
				OP.rollI=tempOP.rollI;
				
				OP.yawP=tempOP.yawP;
				OP.yawD1=tempOP.yawD1;
				OP.yawD2=tempOP.yawD2;
				OP.yawI=tempOP.yawI;

				OP.horiP=tempOP.horiP;
				OP.horiD=tempOP.horiD;
				OP.horiI=tempOP.horiI;

				OP.heightP=tempOP.heightP;
				OP.heightD=tempOP.heightD;
				OP.heightI=tempOP.heightI;
			}
			else if(tempOP.checksum==1)//Neutral
			{
				OP.hoverThrust = tempOP.hoverThrust;
				OP.RCneutral[0] = tempOP.RCneutral[0];
				OP.RCneutral[1] = tempOP.RCneutral[1];
				OP.RCneutral[2] = tempOP.RCneutral[2];
				OP.RCneutral[3] = tempOP.RCneutral[3];
			}
			OP.checksum = CalChecksum(&OP);
			xQueueSend(xDiskWrQueue1,&OP,0);//write to SD card
		}
		
		
		/*update attitude data*/
		xQueueReceive(AHRSToFlightConQueue,&acdt,portMAX_DELAY);
		xstatus=xQueueReceive(INSToFlightConQueue,&pcdt,0);

		InputControl(&odt,&OP);
		//姿态控制的积分项，采用滞回策略开启会关闭
		if(acvt.IntegerEnableOption == 0)
		{
			if(odt.thrustOut>0.5) acvt.IntegerEnableOption=1;
		}
		else if(odt.thrustOut <0.15)
		{
			acvt.IntegerEnableOption = 0;
		}

		PosControl(&odt,&pcdt,&acdt,&OP);	//  ,&pdt,&OP

		AttControl(&acdt,&acvt,&odt,&OP);
		OutputControl(&acvt,&odt,&opt);
		WriteMotor(&opt);

		if(xstatus == pdPASS)
		{	

		}

		if(CNT++>=10)
		{
			CNT=0;
//			sprintf(printf_buffer,"%.2f %.2f %.2f\r\n",acdt.rollAngleRate*57.3,acdt.pitchAngleRate*57.3,acdt.yawAngleRate*57.3);
//			Uart2Send(printf_buffer,strlen(printf_buffer));
		}
		vTaskDelayUntil(&lastTime,(portTickType)(5/portTICK_RATE_MS));
	}
}
