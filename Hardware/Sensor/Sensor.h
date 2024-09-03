#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

//===============================��������=============================
#if 0
#define USE_LOW_USV 8000		//8mSv
#define USE_HIGH_USV 10000		//10mSv
#endif

//yao �л��ٽ��л�ֵ��
#define USE_LOW_USV    2000		//2mSv
#define USE_HIGH_USV   1000		//1mSv

//���л��ٽ��л�ֵ
#define LOW_SEG 0 //�����̶�״̬
#define HIG_SEG 1 //�����̶�״̬


//===============================�ṹ�嶨��===========================




//===============================��������=============================

extern u8 GDoseSeg;
extern float CanshuD;

//===============================��������=============================
void SensorInit(void);
void SensorMeasureBegin(void);
void CaptureSensorPluseCounter(void);
void UseSLParam(float dr);
float Get_Low_Smooth_Counter(void);
float Get_High_Smooth_Counter(void);
float Get_Low_Counter(void);
float Get_High_Counter(void);
void Get_Bendi_Value(char time);
float Netro_Calibration(u32 time,u32 refdata,u8 channel);
u16 CalcAlarmState(LP_SYSTEM_STTAE *me);
#endif
