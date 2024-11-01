#ifndef __FILTER_H
#define __FILTER_H	 
#include "stdio.h"

#define halfT 0.0025					// half the sample period
#define q30  1073741824.0f  

typedef struct
{
	float xv[3];
	float yv[3];
	float input;
	float output;
}Bw30HzLPFTypeDef;

typedef struct
{
	float xv[4];
	float yv[4];
	float input;
	float output;
}Bw50HzLPFTypeDef;

//////////////////////////////////////////////////////////////
// Moving average filtering data struct
typedef struct
{
	float input;
	float buf[5];
	float output;
}MovingAverageFilterTypeDef;

typedef struct
{
	Bw50HzLPFTypeDef GyroxLPF;
	Bw50HzLPFTypeDef GyroyLPF;
	Bw50HzLPFTypeDef GyrozLPF;
	Bw30HzLPFTypeDef AccxLPF;
	Bw30HzLPFTypeDef AccyLPF;
	Bw30HzLPFTypeDef AcczLPF;
	MovingAverageFilterTypeDef GyroxFinal;
	MovingAverageFilterTypeDef GyroyFinal;
	MovingAverageFilterTypeDef GyrozFinal;
} Filter6axisTypeDef;


extern Filter6axisTypeDef Filters;

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF);
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF);
void MovingAverageFilter(MovingAverageFilterTypeDef* pFilter);
void LPFUpdate6axis(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);
void IMUReset(void);

#endif
