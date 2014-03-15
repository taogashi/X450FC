#ifndef _AHRS_H_
#define _AHRS_H_

#include "HAL_AHRS.h"

#define AHRS_FRAME_LEN 28
#define USE_AHRS_ATT_FRAME 1

#define AHRS_SPI_CS_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define AHRS_SPI_CS_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_12)

typedef struct
{
	float gyr[3];
	float acc[3];
	s16	  mag[3];
	float quaternion[4];
	float height;
}SensorDataType;

typedef struct{
	s16 data[9];
	float height;
	s32 Check;
}ComType;

typedef struct{
	float quaternion[4];
	float gyr[3];
}AttDataType;

typedef struct{
	s16 data[10];
	float height;
	s32 Check;
}AttComType;

typedef struct{
	s16 baroheight;
	u8 check;
}BaroHeightType;

void AHRS_SPI_Config(void);
u8 AHRS_SPI_SendByte(u8 byte);
u8 AHRS_SPI_ReadByte(void);
void Delay_us(__IO uint32_t nCount);

u8 ReadAHRSRaw(SensorDataType* sd);

#endif

