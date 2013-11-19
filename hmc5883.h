//��ƽ̨�޹�
//�궨�崫����I2C��ַ���ڲ��Ĵ�����ַ���������ò����Լ�����������
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HMC5883_H__
#define __HMC5883_H__

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define HMC5883_I2C_ADD   0x3C//������ַ

#define HMC5883_CON_A     0x00
#define HMC5883_CON_B     0x01
#define HMC5883_MODE      0x02
#define HMC5883_OUT_X_MSB 0x03
#define HMC5883_OUT_X_LSB 0x04
#define HMC5883_OUT_Z_MSB 0x05
#define HMC5883_OUT_Z_LSB 0x06
#define HMC5883_OUT_Y_MSB 0x07
#define HMC5883_OUT_Y_LSB 0x08
#define HMC5883_STATUS    0x09
#define HMC5883_ID_A      0x0A
#define HMC5883_ID_B      0x0B
#define HMC5883_ID_C      0x0C

typedef enum
{ Sample_Num_1 = 0x00,//Default 1
  Sample_Num_2 = 0x20,//2
  Sample_Num_4 = 0x40,//4
  Sample_Num_8 = 0x60,//8
}SampleNum_TypeDef;	  //�������������ƽ��ֵ

typedef enum
{ Output_Rate_d75 = 0x00,//0.75Hz
  Output_Rate_1d5 = 0x04,//1.5Hz
  Output_Rate_003 = 0x08,//3Hz
  Output_Rate_7d5 = 0x0C,//7.5Hz
  Output_Rate_015 = 0x10,//Default 15Hz
  Output_Rate_030 = 0x14,//30Hz
  Output_Rate_075 = 0x18,//75Hz
  Output_Rate_Res = 0x1C //Reserved
}OutputRate_TypeDef;   //HMC5883 �������

typedef enum
{ Measure_Mode_Normal   = 0x00,//Normal measurement configuration(Default)
  Measure_Mode_Postive  = 0x01,//a positive current is forced across the resistive load for all three axes
  Measure_Mode_Negative = 0x02,//a negative current is forced across the resistive load for all three axes
  Measure_Mode_Reserved = 0x03,//Reserved
}MeasureMode_TypeDef;	//HMC5883 ����ģʽ

typedef enum
{ Gain_Configure_088 = 0x00,//-0.88Ga to +0.88Ga
  Gain_Configure_130 = 0x20,//-1.3Ga  to +1.3Ga
  Gain_Configure_190 = 0x40,//
  Gain_Configure_250 = 0x60,//
  Gain_Configure_400 = 0x80,//
  Gain_Configure_470 = 0xA0,//
  Gain_Configure_560 = 0xC0,//
  Gain_Configure_810 = 0xE0,//ͬ��
}GainConfigure_TypeDef;	//HMC5883 ����ѡ��

typedef enum
{ Operating_Mode_Continuous = 0x00,//��������ģʽ
  Operating_Mode_Single     = 0x01,//���β���ģʽ��Default��
  Operating_Mode_Idle       = 0x02,//Idle Mode ����
  Operating_Mode_Idle2       = 0x03,//Idle Mode
}OperatingMode_TypeDef;	//HMC5883 ����ģʽ

typedef struct
{ SampleNum_TypeDef	    Sample_Num;     //��������
  OutputRate_TypeDef    Output_Rate;    //�������
  MeasureMode_TypeDef   Measure_Mode;   //����ģʽ
  GainConfigure_TypeDef Gain_Configure; //����ѡ��
  OperatingMode_TypeDef Operating_Mode; //����ģʽ 
}HMC5883_InitTypeDef;	 //HMC5883 ��ʼ������

void HMC5883_I2C_Init(void);
void HMC5883_I2C_ByteWrite(u8 slAddr, u8* pBuffer, u8 WriteAddr);
void HMC5883_I2C_BufferRead(u8 slAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void HMC5883_Init(HMC5883_InitTypeDef *HMC5883_InitStructure);  	 //�����̵ĳ�ʼ��

void HMC5883_Read_ID(u8* pid);     //ȷ�ϴ�����ID
u8 HMC5883_Read_Raw(s16* mag);   //��ȡ����������
void HMC5883_Read_mag(s16* mag);

#endif /* __HMC5883_H */

/******************* (C) COPYRIGHT 2012 Skyworks *******************/


