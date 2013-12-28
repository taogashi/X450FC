#include "AHRS.h"
#include "math.h"

#define Dummy_Byte 0xff

u8 spi_byte_buffer[50]={0};
u8 spi_req_header[] = {0x51};
volatile u8 frame_captured = 1;

void AHRS_SPI_Config(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
#ifdef AHRS_SPI_INT_MODE
	NVIC_InitTypeDef NVIC_InitStructure;
#endif

	RCC_APB1PeriphClockCmd(AHRS_SPI_RCC_Periph,ENABLE);
	RCC_AHB1PeriphClockCmd(AHRS_SPI_RCC_Port,ENABLE);

	GPIO_PinAFConfig(AHRS_SPI_NSS_Pin_Port, AHRS_SPI_NSS_Pin_Source, AHRS_SPI_NSS_AF);
	GPIO_PinAFConfig(AHRS_SPI_CLK_Pin_Port, AHRS_SPI_CLK_Pin_Source, AHRS_SPI_CLK_AF);
	GPIO_PinAFConfig(AHRS_SPI_MISO_Pin_Port, AHRS_SPI_MISO_Pin_Source, AHRS_SPI_MISO_AF);
	GPIO_PinAFConfig(AHRS_SPI_MOSI_Pin_Port, AHRS_SPI_MOSI_Pin_Source, AHRS_SPI_MOSI_AF);

	//MISO PB14 | MOSI PB15 | SCK PB13 | NSS PB12
	GPIO_InitStructure.GPIO_Pin =AHRS_SPI_CLK_Pin | AHRS_SPI_MISO_Pin | AHRS_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(AHRS_SPI_NSS_Pin_Port,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AHRS_SPI_NSS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(AHRS_SPI_NSS_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(AHRS_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = AHRS_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(AHRS_SPI, &SPI_InitStructure);

	SPI_Cmd(AHRS_SPI, ENABLE);
	AHRS_SPI_CS_HIGH(); 

#ifdef AHRS_SPI_INT_MODE
	NVIC_InitStructure.NVIC_IRQChannel = AHRS_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = AHRS_INT_PRIOR;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	AHRS_SPI_CS_LOW();
	SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, ENABLE);
#endif
}

u8 AHRS_SPI_SendByte(u8 byte)
{
    while(SPI_I2S_GetFlagStatus(AHRS_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(AHRS_SPI,byte);
	while(SPI_I2S_GetFlagStatus(AHRS_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(AHRS_SPI);
}

u8 AHRS_SPI_ReadByte(void)
{
	return (AHRS_SPI_SendByte(Dummy_Byte));
}

void Delay_us(__IO uint32_t nCount)
{
	u8 k;
	while(nCount--)
	{
		k=28;
		while(k--);
	}
}

u8 ReadAHRSRaw(SensorDataType* sd)
{
	static u8 j=0;
	u8 i;
	ComType cmt;
	u8 bbstatus=0;
	s32 CheckSum=0;
	u8 ret=0;
	static u32 hit=0;
	static u32 miss=0;

#ifndef AHRS_SPI_INT_MODE
	u8 byteToRead = 26;//sizeof(ComType);
	
	AHRS_SPI_CS_LOW();
	Delay_us(5);

	for(i=0;i<byteToRead;i++)
	{
		spi_byte_buffer[i]=AHRS_SPI_SendByte(j++);
		if(spi_byte_buffer[i] == 0xbb)
		{
			bbstatus ++;
			if(bbstatus >= 2)
			{
				bbstatus = 0;
				break;
			}
		}
		else
		{
			bbstatus = 0;
		}
		Delay_us(4);
	}
	AHRS_SPI_CS_HIGH();	
#else
	if(frame_captured == 0)
		return 0;
#endif
   	
	cmt=*(ComType *)spi_byte_buffer;
	for(i=0;i<9;i++) CheckSum+=cmt.data[i];
	if(CheckSum == cmt.Check && CheckSum != 0)
	{
		for(i=0;i<3;i++) sd->gyr[i]=cmt.data[i]*0.00025;
		for(i=0;i<3;i++) sd->acc[i]=cmt.data[i+3]*0.001;
		for(i=0;i<3;i++) sd->mag[i]=cmt.data[i+6];
		hit++;
		ret = 1;
	}
	else
		miss ++;
	
#ifdef AHRS_SPI_INT_MODE
	frame_captured = 0;
	AHRS_SPI_CS_LOW();
	SPI_I2S_SendData(AHRS_SPI, 0x51);
	SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, ENABLE);
#endif
	return ret;
}

u8 ReadAHRSAtt(AttDataType* att_data)
{
	u8 i;
	AttComType act;
	u8 byteBuffer[50];
	u8 byteToRead=sizeof(AttComType);
	s32 CheckSum=0;

	AHRS_SPI_CS_LOW();
	Delay_us(80);
	AHRS_SPI_SendByte(0xa9);
	Delay_us(200);
	for(i=0;i<byteToRead;i++)
	{
	   byteBuffer[i]=AHRS_SPI_ReadByte();
	   Delay_us(4);
	}
	AHRS_SPI_CS_HIGH();	
   	
	act = *(AttComType *)byteBuffer;
	for(i=0;i<7;i++) CheckSum+=act.data[i];
	if(CheckSum == act.Check && CheckSum!=0)
	{
	 	for(i=0;i<4;i++) att_data->quaternion[i]=act.data[i]*0.00025;
		for(i=0;i<3;i++) att_data->gyr[i]=act.data[i+3]*0.00025;
		return 1;
	}
	
	return 0;	
}


u8 ReadBaroHeight(float *height)
{
	u8 i;
	BaroHeightType bht;
	u8 byteBuffer[50];
	u8 byteToRead=sizeof(BaroHeightType);
	u8 CheckSum=0;
	
	AHRS_SPI_CS_LOW();
	Delay_us(80);
	AHRS_SPI_SendByte(0x32);
	Delay_us(200);
	for(i=0;i<byteToRead;i++)
	{
		byteBuffer[i]=AHRS_SPI_ReadByte();
		Delay_us(4);		
	}
	AHRS_SPI_CS_HIGH();

	bht=*(BaroHeightType *)byteBuffer;
	CheckSum = *(u8 *)(&(bht.baroheight))+*((u8 *)(&(bht.baroheight))+1);
	if(CheckSum == bht.check && CheckSum!=0)
	{
	 	*height=bht.baroheight/100.0;
		return 1;
	}

	return 0;
}

void AHRS_IRQHandler(void)
{
	static u8 spi_com_stage = 0;
	static u8 bbstatus = 0;
	
	if(SPI_I2S_GetITStatus(AHRS_SPI, SPI_I2S_IT_RXNE) == SET)
	{
		spi_byte_buffer[spi_com_stage] = SPI_I2S_ReceiveData(AHRS_SPI);
		if(spi_byte_buffer[spi_com_stage] == 0xbb)
		{
			bbstatus ++;
			if(bbstatus >= 2)
			{
				bbstatus = 0;
				SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, DISABLE);
				AHRS_SPI_CS_HIGH();
				spi_com_stage = 0;
				frame_captured = 1;
				return;
			}
		}
		else
		{
			bbstatus = 0;
		}
		SPI_I2S_SendData(AHRS_SPI, Dummy_Byte);
		spi_com_stage++;
//		if(spi_com_stage == 0)
//		{
//			SPI_I2S_ReceiveData(AHRS_SPI);
//			Delay_us(40);
//		}
//		else if(spi_com_stage<AHRS_FRAME_LEN)
//		{
//			spi_byte_buffer[spi_com_stage-1] = SPI_I2S_ReceiveData(AHRS_SPI);
//		}
//		else
//		{
//			spi_byte_buffer[spi_com_stage-1] = SPI_I2S_ReceiveData(AHRS_SPI);
//			SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, DISABLE);
//			AHRS_SPI_CS_HIGH();
//			spi_com_stage = 0;
//			frame_captured = 1;
//			return;
//		}

//		SPI_I2S_SendData(AHRS_SPI, Dummy_Byte);
//		spi_com_stage++;
	}
}

//u8 ReadBaroHeight(float *height)
//{
//	u8 i;
//	baroComType bCT;
//	u8 byteToRead=sizeof(baroComType);
//	s16 CheckSum=0;
//	
//	AHRS_SPI_CS_LOW();
//	Delay_us(330);
//	AHRS_SPI_SendByte(0x32);
//	Delay_us(20);
//	for(i=0;i<byteToRead;i++)
//	{
//		byteBuffer[i]=AHRS_SPI_ReadByte();
//		Delay_us(4);		
//	}
//	AHRS_SPI_CS_HIGH();
//
//	bCT=*(baroComType *)byteBuffer;
//	CheckSum = *(u8 *)(&(bCT.height))+*((u8 *)(&(bCT.height))+1);
//	if(CheckSum == bCT.Check && CheckSum!=0)
//	{
//	 	*height=bCT.height/100.0;
//		if(fabs(*height - baroGFT.pool[(baroGFT.head+1)%baroGFT.length])<3)
//		*height = GaussianFilter(&baroGFT,*height);
//		return 1;
//	}
//
//	return 0;
//}

