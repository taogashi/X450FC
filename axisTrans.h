#ifndef _AXISTRANS_H_
#define _AXISTRANS_H_

#include "stm32f4xx.h"

void Quat2Angle(float *angle,float *quat);
void Angle2Quat(float *quat,float *angle);
void Quat2dcm(float *DCM,float *quat);
void QuatNormalize(float *quat);//��Ԫ����һ��
void Angle2dcm(float *DCM,float *angle);
void MeasureAngle(float *acc,s16 *mag,float *angle,float *refangle,u8 use_ref);

//��ŷ���������ں���ķ�Χ
void EularAngleRestrict(float *angle);

#endif

