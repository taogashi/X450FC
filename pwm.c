#include "pwm.h"

//1520 1525 1111 1529

//u8  tim2OverflowCNT=0;
//u16 tim2PrevIC2Value=0;
//u16	tim2CurIC2Value=0;
//s16	tim2IC2Width=0;  //定时器2通道2脉宽	 超声波
s16  tim4IC1Width=0;	 //接收机1通道 滚转
s16  tim4IC2Width=0;	 //接收机2通道 俯仰
s16  tim4IC3Width=0;	 //接收机3通道 油门
s16  tim4IC4Width=0;	 //接收机4通道 偏航

u16 tim4PrevIC1Value=0;
u16 tim4PrevIC2Value=0;
u16 tim4PrevIC3Value=0;
u16 tim4PrevIC4Value=0;

u16	tim4CurIC1Value=0;
u16	tim4CurIC2Value=0;
u16	tim4CurIC3Value=0;
u16	tim4CurIC4Value=0;

s16  tim5IC1Width=0;   
s16  tim5IC2Width=0;
s16  tim5IC3Width=0;
s16  tim5IC4Width=0;

u16 tim5PrevIC1Value=0;
u16 tim5PrevIC2Value=0;
u16 tim5PrevIC3Value=0;
u16 tim5PrevIC4Value=0;

u16	tim5CurIC1Value=0;
u16	tim5CurIC2Value=0;
u16	tim5CurIC3Value=0;
u16	tim5CurIC4Value=0;

void TIM2_Config(void) //PWM正脉宽捕获
{
	uint16_t CCR1_Val = 1500;
	uint16_t CCR2_Val = 1500;
//	uint16_t CCR3_Val = 1500;
//	uint16_t CCR4_Val = 1500;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	//使能定时器2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;//| GPIO_Pin_10 | GPIO_Pin_11;				//定时器2通道2，管脚PA0
   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

	TIM_BaseInitStructure.TIM_Period = 20000; //20ms       //定时器2计时周期   
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock/2) / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 

	/*************************** 通道1 ********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //PWM功能使能
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                            //写比较值(占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   //置高
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	/****************************** 通道2 ******************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	/******************************* 通道3 *********************************/
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	
//	/****************************** 通道4 *********************************/
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);                        //
	TIM_Cmd(TIM2, ENABLE);                  
}

void TIM3_Config(void)
{
	uint16_t CCR1_Val = 100;
	uint16_t CCR2_Val = 100;
	uint16_t CCR3_Val = 100;
	uint16_t CCR4_Val = 100;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 	//使能定时器2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	TIM_BaseInitStructure.TIM_Period = 2000; //5ms       //定时器3计时周期   
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock/2) / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
    TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure); 

	/*************************** 通道1 ********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //PWM功能使能
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                            //写比较值(占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   //置高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/****************************** 通道2 ******************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/******************************* 通道3 *********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/****************************** 通道4 *********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);                        //
	TIM_Cmd(TIM3, ENABLE);  
}

void TIM4_Config(void) //PWM正脉宽捕获
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_ICInitTypeDef  TIM_ICInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
   	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

	TIM_BaseInitStructure.TIM_Period = 40000; //40ms          
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock/2) / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 
   	
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); 
  	TIM_Cmd(TIM4, ENABLE);    
}

void TIM4_IT_Config(void)	//定时器4中断
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler(void)
{	
	s16 temp[5]={0};
	if(TIM_GetITStatus(TIM4,TIM_IT_CC1)==SET)//定时器4通道1
	{
		TIM_ITConfig(TIM4, TIM_IT_CC1, DISABLE);
		if(!(TIM4->CCER & 0x0002)) //上升沿触发
		{
			tim4PrevIC1Value = TIM_GetCapture1(TIM4);
			TIM4->CCER |= 0x0002;		
		}
		else
		{
			tim4CurIC1Value = TIM_GetCapture1(TIM4);
			temp[1] = tim4CurIC1Value-tim4PrevIC1Value;
			if(temp[1]<0)
			temp[1]+=40000;
			if(temp[1]>900 && temp[1]<2100) tim4IC1Width=temp[1];
			TIM4->CCER &= 0xfffd;			
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	}

	if(TIM_GetITStatus(TIM4,TIM_IT_CC2)==SET)	  //定时器4通道2
	{
		TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
		if(!(TIM4->CCER & 0x0020)) //上升沿触发
		{
			tim4PrevIC2Value = TIM_GetCapture2(TIM4);
			TIM4->CCER |= 0x0020;		
		}
		else
		{
			tim4CurIC2Value = TIM_GetCapture2(TIM4);
			temp[2] = tim4CurIC2Value-tim4PrevIC2Value;
			if(temp[2]<0)
			temp[2]+=40000;
			if(temp[2]>900 && temp[2]<2100) tim4IC2Width=temp[2];
			TIM4->CCER &= 0xffdf;			
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	}

	if(TIM_GetITStatus(TIM4,TIM_IT_CC3)==SET)	  //定时器4通道3
	{
		TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);
		if(!(TIM4->CCER & 0x0200)) //上升沿触发
		{
			tim4PrevIC3Value = TIM_GetCapture3(TIM4);
			TIM4->CCER |= 0x0200;		
		}
		else
		{
			tim4CurIC3Value = TIM_GetCapture3(TIM4);
			temp[3] = tim4CurIC3Value-tim4PrevIC3Value;
			if(temp[3]<0)
			temp[3]+=40000;
			if(temp[3]>900 && temp[3]<2100) tim4IC3Width=temp[3];
			TIM4->CCER &= 0xfdff;			
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
		TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	}
	
	if(TIM_GetITStatus(TIM4,TIM_IT_CC4)==SET)	  //定时器4通道4
	{
		TIM_ITConfig(TIM4, TIM_IT_CC4, DISABLE);
		if(!(TIM4->CCER & 0x2000)) //上升沿触发
		{
			tim4PrevIC4Value = TIM_GetCapture4(TIM4);
			TIM4->CCER |= 0x2000;		
		}
		else
		{
			tim4CurIC4Value = TIM_GetCapture4(TIM4);
			temp[4] = tim4CurIC4Value-tim4PrevIC4Value;
			if(temp[4]<0)
			temp[4]+=40000;
			if(temp[4]>900 && temp[4]<2100) tim4IC4Width=temp[4];
			TIM4->CCER &= 0xdfff;			
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
		TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
	}	   	
}

void TIM5_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_ICInitTypeDef  TIM_ICInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
   	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	TIM_BaseInitStructure.TIM_Period = 40000; //40ms          
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock/2) / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM5, &TIM_BaseInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM5, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM5, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM5, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM5, &TIM_ICInitStructure); 
   	
	TIM_ITConfig(TIM5, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); 
  	TIM_Cmd(TIM5, ENABLE);   
}

void TIM5_IT_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM5_IRQHandler(void)
{	
	s16 temp[5]={0};
	if(TIM_GetITStatus(TIM5,TIM_IT_CC1)==SET)//定时器4通道1
	{
		TIM_ITConfig(TIM5, TIM_IT_CC1, DISABLE);
		if(!(TIM5->CCER & 0x0002)) //上升沿触发
		{
			tim5PrevIC1Value = TIM_GetCapture1(TIM5);
			TIM5->CCER |= 0x0002;		
		}
		else
		{
			tim5CurIC1Value = TIM_GetCapture1(TIM5);
			temp[1] = tim5CurIC1Value-tim5PrevIC1Value;
			if(temp[1]<0)
			temp[1]+=40000;
			if(temp[1]>900 && temp[1]<2100) tim5IC1Width=temp[1];
			TIM5->CCER &= 0xfffd;			
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
	}

	if(TIM_GetITStatus(TIM5,TIM_IT_CC2)==SET)	  //定时器4通道2
	{
		TIM_ITConfig(TIM5, TIM_IT_CC2, DISABLE);
		if(!(TIM5->CCER & 0x0020)) //上升沿触发
		{
			tim5PrevIC2Value = TIM_GetCapture2(TIM5);
			TIM5->CCER |= 0x0020;		
		}
		else
		{
			tim5CurIC2Value = TIM_GetCapture2(TIM5);
			temp[2] = tim5CurIC2Value-tim5PrevIC2Value;
			if(temp[2]<0)
			temp[2]+=40000;
			if(temp[2]>900 && temp[2]<2100) tim5IC2Width=temp[2];
			TIM5->CCER &= 0xffdf;			
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
		TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
	}

	if(TIM_GetITStatus(TIM5,TIM_IT_CC3)==SET)	  //定时器4通道3
	{
		TIM_ITConfig(TIM5, TIM_IT_CC3, DISABLE);
		if(!(TIM5->CCER & 0x0200)) //上升沿触发
		{
			tim5PrevIC3Value = TIM_GetCapture3(TIM5);
			TIM5->CCER |= 0x0200;		
		}
		else
		{
			tim5CurIC3Value = TIM_GetCapture3(TIM5);
			temp[3] = tim5CurIC3Value-tim5PrevIC3Value;
			if(temp[3]<0)
			temp[3]+=40000;
			if(temp[3]>900 && temp[3]<2100) tim5IC3Width=temp[3];
			TIM5->CCER &= 0xfdff;			
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
		TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
	}
	
	if(TIM_GetITStatus(TIM5,TIM_IT_CC4)==SET)	  //定时器4通道4
	{
		TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
		if(!(TIM5->CCER & 0x2000)) //上升沿触发
		{
			tim5PrevIC4Value = TIM_GetCapture4(TIM5);
			TIM5->CCER |= 0x2000;		
		}
		else
		{
			tim5CurIC4Value = TIM_GetCapture4(TIM5);
			temp[4] = tim5CurIC4Value-tim5PrevIC4Value;
			if(temp[4]<0)
			temp[4]+=40000;
			if(temp[4]>900 && temp[4]<2100) tim5IC4Width=temp[4];
			TIM5->CCER &= 0xdfff;			
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
		TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
	}	   	
}

void Disable_Timer_IT(void)
{
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, DISABLE); 
	TIM_ITConfig(TIM5, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, DISABLE); 
}

void Enable_Timer_IT(void)
{
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); 
	TIM_ITConfig(TIM5, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); 
}
//void TIM2_Config(void) //PWM正脉宽捕获
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
//	TIM_ICInitTypeDef  TIM_ICInitStructure;
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	//使能定时器2时钟
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				//定时器2通道2，管脚PA0
//   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//   	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
//
//	TIM_BaseInitStructure.TIM_Period = 20000; //20ms       //定时器2计时周期   
//    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1; //1us       
//    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
//    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
//    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
//
////--------------------------------------通道2，输入捕获------------------------------------------
//   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 		   //定时器2通道2输入捕获模式
//   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
//   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
//   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
//   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
//   	TIM_ICInit(TIM2, &TIM_ICInitStructure); 
//
//	TIM_ITConfig(TIM2, TIM_IT_CC2 | TIM_IT_Update, ENABLE); 
//  	TIM_Cmd(TIM2, ENABLE); 
//}

//void TIM2_IT_Config(void)	//定时器2中断
//{
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	/* Enable the TIM3 global Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}
//
//void TIM2_IRQHandler(void)
//{	
//	s16 temp;
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
//	{
//		tim2OverflowCNT++;
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		
//	}
//	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)==SET)	  //定时器2通道1
//	{
//		TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
//		if(!(TIM2->CCER & 0x0020)) //上升沿触发
//		{
//			tim2OverflowCNT=0;
//			tim2PrevIC2Value = TIM_GetCapture2(TIM2);
//			TIM2->CCER |= 0x0020;		
//		}
//		else
//		{
//			tim2CurIC2Value = TIM_GetCapture2(TIM2);
//			temp = tim2CurIC2Value+tim2OverflowCNT*20000-tim2PrevIC2Value;
//
//			/*200 表示 0.2毫秒 对应距离3厘米
//			  25000表示 25毫秒 对应距离4米
//			  懂了吧？*/
//			if(temp>200 && temp<25000) tim2IC2Width=temp;
//			TIM2->CCER &= 0xffdf;			
//		}
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
//		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
//	}	   	
//}
//
//float GetSonicDistance(void)
//{
//	float dist=(float)tim2IC2Width*0.017;
//	return dist;
//}
//
//float GetVerticSpeed(void)
//{
//	static s16 lastWidth=0;
//	static float V_Speed[3]={0.0};
//	float deltaT=0.0;
//
//	if(lastWidth>0)
//	{
//		if(tim2IC2Width != lastWidth)
//		{
//			deltaT=	(float)GetSystikCount(SONICCNT)/CONTROLFRE;
//			V_Speed[0] = (float)(tim2IC2Width-lastWidth)*0.017/deltaT;
//		}
//		else if(V_Speed[0]<0.5)
//		{
//			V_Speed[0]=0.0;
//		}
//	}
//	V_Speed[0]=0.3*V_Speed[2]+0.6*V_Speed[1]+0.1*V_Speed[0];
//	V_Speed[2]=V_Speed[1];
//	V_Speed[1]=V_Speed[0];
//	lastWidth=tim2IC2Width;
//	ResetSystikCount(SONICCNT);
//	return V_Speed[0];
//}
