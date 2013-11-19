#include "AHRS.h"
#include "math.h"

#define Dummy_Byte 0xff

u8 byteBuffer[50];

void AHRS_SPI_Config(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

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

u8 ReadAHRS(SensorDataType* sd)
{
	u8 i;
	ComType cmt;
	u8 byteToRead=sizeof(ComType);
	s32 CheckSum=0;

	AHRS_SPI_CS_LOW();
	Delay_us(80);
	AHRS_SPI_SendByte(0x51);
	Delay_us(200);
	for(i=0;i<byteToRead;i++)
	{
	   byteBuffer[i]=AHRS_SPI_ReadByte();
	   Delay_us(4);
	}
	AHRS_SPI_CS_HIGH();	
   	
	cmt=*(ComType *)byteBuffer;
	for(i=0;i<9;i++) CheckSum+=cmt.data[i];
	if(CheckSum == cmt.Check && CheckSum!=0)
	{
	 	for(i=0;i<3;i++) sd->gyr[i]=cmt.data[i]*0.00025;
		for(i=0;i<3;i++) sd->acc[i]=cmt.data[i+3]*0.001;
		for(i=0;i<3;i++) sd->mag[i]=cmt.data[i+6];

		sd->acc[2]+=0.67;
		sd->mag[0]+=88;
		sd->mag[0]*=1.0360;
		sd->mag[1]+=19;
		sd->mag[1]*=0.9941;
		sd->mag[2]+=43;
		sd->mag[2]*=1.0595;

//		for(i=0;i<3;i++)
//			sd->acc[i]=GaussianFilter(&(sensorGFT[i]),sd->acc[i]);
//		for(i=0;i<3;i++)
//			sd->mag[i]=GaussianFilter(&(sensorGFT[i+3]),sd->mag[i]);
		return 1;
	}

	return 0;
}

u8 ReadBaroHeight(float *height)
{
	u8 i;
	BaroHeightType bht;
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

