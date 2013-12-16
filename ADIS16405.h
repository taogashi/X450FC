#ifndef _ADIS16405_H_
#define _ADIS16405_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include "HAL_ADIS16405.h"
	 
#define ADIS16405_FRAME_LEN 24
#define ADIS16405_REQ_LEN 2

#define ADIS16405_SPI_CS_LOW()  GPIO_ResetBits(ADIS16405_SPI_NSS_Pin_Port, ADIS16405_SPI_NSS_Pin)
#define ADIS16405_SPI_CS_HIGH() GPIO_SetBits(ADIS16405_SPI_NSS_Pin_Port, ADIS16405_SPI_NSS_Pin)

#define ADIS16405_XGYRO_OUT ((u8)0x04)
#define ADIS16405_YGYRO_OUT ((u8)0x06)
#define ADIS16405_ZGYRO_OUT ((u8)0x08)

#define ADIS16405_XACCL_OUT ((u8)0x0A)
#define ADIS16405_YACCL_OUT ((u8)0x0C)
#define ADIS16405_ZACCL_OUT ((u8)0x0E)

#define ADIS16405_XMAGN_OUT ((u8)0x10)
#define ADIS16405_YMAGN_OUT ((u8)0x12)
#define ADIS16405_ZMAGN_OUT ((u8)0x14)

#define ADIS16405_SMPL_PRD 	((u8)0x36)
#define ADIS16405_SENS_AVG 	((u8)0x38)
#define ADIS16405_SLP_CNT 	((u8)0x3A)
#define ADIS16405_GLOB_CMD 	((u8)0x3E)
#define ADIS16405_PRODUCT_ID 	((u8)0x56)

typedef struct{
	float gyr[3];
	float acc[3];
	s16 mag[3];
}ADIS16405_DataTypeDef;

void ADIS16405_SPI_Init(void);
u8 ADIS16405_SPI_SendByte(u8 byte);
u8 ADIS16405_SPI_ReadByte(void);
u8 ADIS16405_Burst_Read(ADIS16405_DataTypeDef* adt);
void ADIS16405_Restore_Cali(void);
void ADIS16405_Preci_Cali(void);
void ADIS16405_Reset(void);

void ADIS16405_Delayms(u16 n);
void ADIS16405_Delayus(u16 n);

#endif
