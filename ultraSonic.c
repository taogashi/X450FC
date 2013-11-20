#include "ultraSonic.h"

u8  tim2OverflowCNT=0;
u16 tim2PrevIC4Value=0;
u16	tim2CurIC4Value=0;
s16	tim2IC4Width=0;  //定时器2通道1脉宽	 超声波

u8 DataReady=0;

void TIM2_Config(void) //PWM正脉宽捕获
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	//使能定时器2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);

	TIM_BaseInitStructure.TIM_Period = 4000; //30ms       //定时器2计时周期   
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock/2) / 100000) - 1; //10us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 

	/******************************* 通道3 *********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*************************** 通道4 ********************************/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM2, &TIM_ICInitStructure); 

	
	TIM_ITConfig(TIM2, TIM_IT_CC4 | TIM_IT_Update, ENABLE); 
	TIM_ARRPreloadConfig(TIM3, ENABLE);    
	TIM_Cmd(TIM2, ENABLE);                  
}

void TIM2_IT_Config(void)	//定时器4中断
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
	int temp;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		tim2OverflowCNT++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}

	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)==SET)//定时器4通道1
	{
		TIM_ITConfig(TIM2, TIM_IT_CC4, DISABLE);
		if(!(TIM2->CCER & 0x2000)) //上升沿触发
		{
			tim2PrevIC4Value = TIM_GetCapture4(TIM2);
			TIM2->CCER |= 0x2000;
			tim2OverflowCNT=0;		
		}
		else
		{
			tim2CurIC4Value = TIM_GetCapture4(TIM2);
			temp = tim2CurIC4Value-tim2PrevIC4Value;
			temp +=	tim2OverflowCNT*4000;
			tim2OverflowCNT=0;
			//-------------------------------------------------------------
			if(temp>10 && temp<4000) 
			{
				tim2IC4Width=temp;
				DataReady = 1;
			}
			TIM2->CCER &= 0xdfff;			
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	}
}

u8 GetUltraSonicMeasure(float *dist)
{
	if(DataReady == 0)
		return 0;
	DataReady = 0;
	*dist = (float)tim2IC4Width*0.0024293-0.09286;
	return 1;
}
