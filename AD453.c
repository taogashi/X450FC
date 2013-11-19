#include "AD453.h"
#include "OSConfig.h"

/*
���ã�	��ʼ��SPI1
������	��
����ֵ����
*/
void AD453_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(AD453_SPI_RCC_Periph,ENABLE);
	RCC_AHB1PeriphClockCmd(AD453_SPI_RCC_Port,ENABLE);

	GPIO_PinAFConfig(AD453_SPI_NSS_Pin_Port, AD453_SPI_NSS_Pin_Source, AD453_SPI_NSS_AF);
	GPIO_PinAFConfig(AD453_SPI_CLK_Pin_Port, AD453_SPI_CLK_Pin_Source, AD453_SPI_CLK_AF);
	GPIO_PinAFConfig(AD453_SPI_MISO_Pin_Port, AD453_SPI_MISO_Pin_Source, AD453_SPI_MISO_AF);
	GPIO_PinAFConfig(AD453_SPI_MOSI_Pin_Port, AD453_SPI_MOSI_Pin_Source, AD453_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Pin =AD453_SPI_CLK_Pin | AD453_SPI_MISO_Pin | AD453_SPI_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD453_SPI_NSS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(AD453_SPI_NSS_Pin_Port,&GPIO_InitStructure);

	SPI_Cmd(AD453_SPI, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = AD453_SPI_BaudRatePrescaler;	 //656kbps
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(AD453_SPI, &SPI_InitStructure);
	SPI_Cmd(AD453_SPI, ENABLE);
	AD453_SPI_CS_HIGH(); 
}

/*
���ã�	SPI�����ֽڣ�ͬʱ���ؽ��յ���һ���ֽڣ�
������	Ҫ���͵��ֽ�
����ֵ�����յ����ֽ�
*/
u8 AD453_SPI_SendByte(u8 byte)
{
	RTOS_ENTER_CRITICAL();
    while(SPI_I2S_GetFlagStatus(AD453_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(AD453_SPI,byte);
	while(SPI_I2S_GetFlagStatus(AD453_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	RTOS_EXIT_CRITICAL();
	return SPI_I2S_ReceiveData(AD453_SPI);
}

/*
���ã��Ӵ��豸��ȡһ���ֽ�
��������
����ֵ����ȡ���ֽ�
*/
u8 AD453_SPI_ReadByte(void)
{
	return (AD453_SPI_SendByte(0xff));
}

/*
���ã�	��ADXRS453����һ������
������	����ṹ��
����ֵ�����յ��Ĵ�����response�������κμ��
*/
u32 AD453_Send_Command(AD453_Command* CmdStructure)
{
	u32 cmd32=0;
	u8 wbuffer[4]={0};

	u32 receiveData=0;
	u8 rbuffer[4]={0};

	u8 i=0;	
	u8 delay=10;

	switch(CmdStructure->CMDType)
	{
		case SENSOR_DATA_REQ:
			cmd32=(u32)0x20000000;
			if(CmdStructure->SyncNum&0x01) cmd32&=0x40000000;
			if(CmdStructure->SyncNum&0x02) cmd32&=0x80000000;
			if(CmdStructure->SyncNum&0x04) cmd32&=0x10000000;
			if(CmdStructure->SelfTestEnable==1) cmd32&=0x00000002;
			break;
		case READ_REG:
			cmd32=(u32)0x80000000;
			cmd32 |= ((0x000000ff & CmdStructure->RegAddr)<<17);
			break;
		case WRITE_REG:
			cmd32=(u32)0x40000000;
			cmd32 |= ((0x000000ff & CmdStructure->RegAddr)<<17);
			cmd32 |= ((0x0000ffff & CmdStructure->DatatoSend)<<1);
			break;
	}
	AD453_Cal_ODD_Bit(&cmd32);

	wbuffer[0]=((0xff000000)&cmd32)>>24;
	wbuffer[1]=((0x00ff0000)&cmd32)>>16;
	wbuffer[2]=((0x0000ff00)&cmd32)>>8;
	wbuffer[3]=((0x000000ff)&cmd32);

	AD453_SPI_CS_LOW();
	AD453_Delayus(1);
	for(i=0;i<4;i++)
	{
		rbuffer[i]=AD453_SPI_SendByte(wbuffer[i]);
	}
	AD453_Delayus(1);
	AD453_SPI_CS_HIGH();
	receiveData=rbuffer[0];
	receiveData<<=8;
	receiveData |= rbuffer[1];
	receiveData<<=8;
	receiveData |= rbuffer[2];
	receiveData<<=8;
	receiveData |= rbuffer[3];

	while(delay--);
	return receiveData;
}

/*
���ã�	��ADXRS453��Response�н���������
������	response��out ������ݵ�ָ��
����ֵ��response����
*/
ResponseTypeDef AD453_Respons_Analyse(u32 resp,u16* out)
{
	u8 flag=0;
	u8 dataValid=0;

	//����Ƿ�Ϊ��ʼ��Ӧ0x00000001
	if(resp==1)
		return INIT_RESPONSE;
	
	//��У��
	if(0==AD453_Check_ODD(resp))
		return ODD_CHECK_FAIL;

	//��ȡ����
	flag=(resp & 0xef000000)>>24;
	switch(flag)
	{
		case 0x4e://Read Response
			*out=(resp & 0x001fffe0)>>5;
			return READ;
		case 0x2e://Write Response
			*out=(resp & 0x001fffe0)>>5;
			return WRITE;
		case 0x0e://R/W error response
			*out=0xffff;//indicate an error occur
			return RWERROR;
		default://Sensor Data Response
			dataValid=(resp & 0x0c000000)>>26;
			if(dataValid == 0x01)
			{ 
				*out=(resp & 0x03fffc00)>>10;
				return VALID_DATA;
			}
			else return INVALID_DATA;//indicate an error
	}
}

void AD453_Init(void)
{
}

float AD453_Read_Rate(void)
{
	s16 gyr;
	u16 data;
	float out;

	AD453_Command cmd;
	u32 Resp=0;

	cmd.CMDType=SENSOR_DATA_REQ;
	cmd.SyncNum=0;

	do
	{
		Resp=AD453_Send_Command(&cmd);
	}while(VALID_DATA != AD453_Respons_Analyse(Resp,&data));
	gyr=0;
	gyr |= data;
	out=gyr*0.00021817; //������
	return out;
}

void AD453_Delayms(u16 n)
{
	u16 i;
	while(n--)
	{
		i=28000;
		while(i--);
	}
}

void AD453_Delayus(u16 n)
{
	u16 i;
	while(n--)
	{
		i=28;
		while(i--);
	}	
}

/*
���ã�	���ADXRS453����У��
������	������response
����ֵ��0 У��ʧ�ܣ�1 У��ɹ�
*/
u8 AD453_Check_ODD(u32 data)
{
	u8 i;

	u8 checkSum=0;
	u32 mask=0x80000000;
	
	//����16bit
	for(i=0;i<16;i++)
	{
		checkSum+=((mask&data)!=0);
		mask>>=1;
	}

	//�������bit
	if((checkSum&0x01)!=0)
	{
		checkSum=0;
		mask=1;
		for(i=0;i<32;i++)
		{
			checkSum+=((mask&data)!=0);
			mask<<=1;
		}
		if((checkSum&0x01)!=0) return 1;
		else 
			return 0;
	}
	else 
		return 0;	
}

void AD453_Cal_ODD_Bit(u32* data)
{
	u8 i;
	u8 checkSum=0;
	u32 mask=0x80000000;
	for(i=0;i<31;i++)
	{
		checkSum+=((mask&(*data))!=0);
		mask>>=1;
	}
	if((checkSum&0x01)!=0) *data &= 0xfffffffe;
	else *data |= 0x00000001;	
}
