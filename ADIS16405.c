#include "ADIS16405.h"
#
#include "OSConfig.h"

#define Dummy_Byte 0xff

u8 spi_byte_buffer[50]={0};
u8 spi_req_header[] = {0x3E, 0x00};
volatile u8 frame_captured = 1;

/*
作用：	初始化SPI1
参数：	无
返回值：无
*/
void ADIS16405_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
#ifdef ADIS16405_SPI_INT_MODE
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	

	RCC_APB2PeriphClockCmd(ADIS16405_SPI_RCC_Periph,ENABLE);
	RCC_AHB1PeriphClockCmd(ADIS16405_SPI_RCC_Port,ENABLE);

	GPIO_PinAFConfig(ADIS16405_SPI_NSS_Pin_Port, ADIS16405_SPI_NSS_Pin_Source, ADIS16405_SPI_NSS_AF);
	GPIO_PinAFConfig(ADIS16405_SPI_CLK_Pin_Port, ADIS16405_SPI_CLK_Pin_Source, ADIS16405_SPI_CLK_AF);
	GPIO_PinAFConfig(ADIS16405_SPI_MISO_Pin_Port, ADIS16405_SPI_MISO_Pin_Source, ADIS16405_SPI_MISO_AF);
	GPIO_PinAFConfig(ADIS16405_SPI_MOSI_Pin_Port, ADIS16405_SPI_MOSI_Pin_Source, ADIS16405_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Pin =ADIS16405_SPI_CLK_Pin | ADIS16405_SPI_MISO_Pin | ADIS16405_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ADIS16405_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ADIS16405_SPI_NSS_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(ADIS16405_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = ADIS16405_SPI_BaudRatePrescaler;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(ADIS16405_SPI, &SPI_InitStructure);

	SPI_Cmd(ADIS16405_SPI, ENABLE);
	ADIS16405_SPI_CS_HIGH(); 

#ifdef ADIS16405_SPI_INT_MODE
	
	NVIC_InitStructure.NVIC_IRQChannel = ADIS16405_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADIS16405_INT_PRIOR;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/*
作用：	SPI发送字节（同时返回接收到的一个字节）
参数：	要发送的字节
返回值：接收到的字节
*/
u8 ADIS16405_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(ADIS16405_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(ADIS16405_SPI,byte);
	while(SPI_I2S_GetFlagStatus(ADIS16405_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(ADIS16405_SPI);
}

/*
作用：从从设备读取一个字节
参数：无
返回值：读取的字节
*/
u8 ADIS16405_SPI_ReadByte(void)
{
	return (ADIS16405_SPI_SendByte(0xff));
}

u8 ADIS16405_Burst_Read(ADIS16405_DataTypeDef* adt)
{
	s16 wordBuffer[9]={0};
	u8 i;
#ifndef ADIS16405_SPI_INT_MODE
	ADIS16405_SPI_CS_LOW();
	ADIS16405_Delayus(1);
	ADIS16405_SPI_SendByte(0x3E);
	ADIS16405_SPI_SendByte(0x00);
	ADIS16405_Delayus(1);
	for(i=0;i<24;i++)
	{
		spi_byte_buffer[i] = ADIS16405_SPI_ReadByte();
	}
	ADIS16405_Delayus(1);
	ADIS16405_SPI_CS_HIGH();
#else
	if(frame_captured != 1)
		return 0;
#endif
	
	for(i=0;i<9;i++)
	{
		wordBuffer[i]=(spi_byte_buffer[i*2+2]&0x003f)<<8 | (spi_byte_buffer[i*2+3]);
	}		
	for(i=0;i<9;i++)
	{
		if(wordBuffer[i]>8192)
			wordBuffer[i] -= 2*8192;		
	}

	adt->gyr[0]=-0.0008727*wordBuffer[0];//0.05*wordBuffer[i];
	adt->gyr[1]=0.0008727*wordBuffer[1];//0.05*wordBuffer[i];
	adt->gyr[2]=-0.0008727*wordBuffer[2];//0.05*wordBuffer[i];
	
	adt->acc[0]=0.03271*wordBuffer[3];//0.00333*wordBuffer[i+3];
	adt->acc[1]=-0.03271*wordBuffer[4];//0.00333*wordBuffer[i+3];
	adt->acc[2]=0.03271*wordBuffer[5];//0.00333*wordBuffer[i+3];

	adt->mag[0]=-0.5*wordBuffer[6];
	adt->mag[1]=0.5*wordBuffer[7];
	adt->mag[2]=-0.5*wordBuffer[8];
	
#ifdef ADIS16405_SPI_INT_MODE
	frame_captured = 0;
	ADIS16405_SPI_CS_LOW();
	SPI_I2S_SendData(ADIS16405_SPI, spi_req_header[0]);
	SPI_I2S_ITConfig(ADIS16405_SPI, SPI_I2S_IT_RXNE, ENABLE);
#endif

	return 1;
}

void ADIS16405_Preci_Cali(void)
{
	ADIS16405_SPI_CS_LOW();
	ADIS16405_Delayus(1);
	ADIS16405_SPI_SendByte(0xBE);
	ADIS16405_SPI_SendByte(0x10);
	ADIS16405_Delayus(1);
	ADIS16405_SPI_CS_LOW();	
	//need 30sec
}

void ADIS16405_Reset(void)
{
	ADIS16405_SPI_CS_LOW();
	ADIS16405_Delayus(1);
	ADIS16405_SPI_SendByte(0xBE);
	ADIS16405_SPI_SendByte(0x80);
	ADIS16405_Delayus(1);
	ADIS16405_SPI_CS_LOW();	
	//need 50ms
}

void ADIS16405_Restore_Cali(void)
{
	ADIS16405_SPI_CS_LOW();
	ADIS16405_Delayus(1);
	ADIS16405_SPI_SendByte(0xBE);
	ADIS16405_SPI_SendByte(0x02);
	ADIS16405_Delayus(1);
	ADIS16405_SPI_CS_LOW();	
	//need 50ms	
}

void ADIS16405_Delayms(u16 n)
{
	u16 i;
	while(n--)
	{
		i=28000;
		while(i--);
	}
}

void ADIS16405_Delayus(u16 n)
{
	u16 i;
	while(n--)
	{
		i=28;
		while(i--);
	}	
}

void ADIS16405_IRQHandler(void)
{
	static u8 spi_com_stage = 0;
	
	if(SPI_I2S_GetITStatus(ADIS16405_SPI, SPI_I2S_IT_RXNE) == SET)
	{
		if(spi_com_stage < ADIS16405_REQ_LEN-1)
		{
			SPI_I2S_ReceiveData(ADIS16405_SPI);
			SPI_I2S_SendData(ADIS16405_SPI, spi_req_header[spi_com_stage+1]);
		}
		else if(spi_com_stage == ADIS16405_REQ_LEN-1)
		{
			SPI_I2S_ReceiveData(ADIS16405_SPI);
			SPI_I2S_SendData(ADIS16405_SPI, Dummy_Byte);
		}
		else if(spi_com_stage<ADIS16405_FRAME_LEN+ADIS16405_REQ_LEN-1)
		{
			spi_byte_buffer[spi_com_stage-ADIS16405_REQ_LEN] = SPI_I2S_ReceiveData(ADIS16405_SPI);
			SPI_I2S_SendData(ADIS16405_SPI, Dummy_Byte);
		}
		else
		{
			spi_byte_buffer[spi_com_stage-ADIS16405_REQ_LEN] = SPI_I2S_ReceiveData(ADIS16405_SPI);
			SPI_I2S_ITConfig(ADIS16405_SPI, SPI_I2S_IT_RXNE, DISABLE);
			ADIS16405_SPI_CS_HIGH();
			spi_com_stage = 0;
			frame_captured = 1;
			return;
		}

		spi_com_stage++;
	}
}
