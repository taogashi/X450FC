#ifndef _AD453_H_
#define _AD453_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include "HAL_AD453.h"

#define AD453_SPI_CS_LOW()  GPIO_ResetBits(AD453_SPI_NSS_Pin_Port, AD453_SPI_NSS_Pin)
#define AD453_SPI_CS_HIGH() GPIO_SetBits(AD453_SPI_NSS_Pin_Port, AD453_SPI_NSS_Pin)

#define AD453_RATE1 ((u8)0x00)
#define AD453_RATE0 ((u8)0x01)

#define AD453_TEMP1 ((u8)0x02)
#define AD453_TEMP0 ((u8)0x03)

#define AD453_LOCST1 ((u8)0x04)
#define AD453_LOCST0 ((u8)0x05)

#define AD453_HICST1 ((u8)0x06)
#define AD453_HICST0 ((u8)0x07)

#define AD453_QUAD1 ((u8)0x08)
#define AD453_QUAD0 ((u8)0x09)

#define AD453_FAULT1 ((u8)0x0A)
#define AD453_FAULT0 ((u8)0x0B)

#define AD453_PID1 ((u8)0x0C)
#define AD453_PID0 ((u8)0x0D)

#define AD453_SN3 ((u8)0x0E)
#define AD453_SN2 ((u8)0x0F)
#define AD453_SN1 ((u8)0x10)
#define AD453_SN0 ((u8)0x11)

typedef enum{
	SENSOR_DATA_REQ,
	READ_REG,
	WRITE_REG
}CommandTypeDef;

typedef enum{
	ODD_CHECK_FAIL,
	INIT_RESPONSE,
	VALID_DATA,
	INVALID_DATA,
	READ,
	WRITE,
	RWERROR
}ResponseTypeDef;

typedef struct{
	CommandTypeDef CMDType;
	u8 SyncNum;
	u8 RegAddr;
	u16 DatatoSend;
	u8 SelfTestEnable;
}AD453_Command;

void AD453_SPI_Init(void);
u8 AD453_SPI_SendByte(u8 byte);
u8 AD453_SPI_ReadByte(void);
u32 AD453_Send_Command(AD453_Command* CmdStructure);
ResponseTypeDef AD453_Respons_Analyse(u32 resp,u16* out);
float AD453_Read_Rate(void);
//u8 AD453_Read_State(void);
//void AD453_Read_RawData(s16* out);
//void AD453_Read_GyroRate(float* out);
//void AD453_Cali(void);

void AD453_Delayms(u16 n);
void AD453_Delayus(u16 n);
u8 AD453_Check_ODD(u32 data);
void AD453_Cal_ODD_Bit(u32* data);

#endif
