#ifndef __PID_H
#define __PID_H	 
#include "stdio.h"
#include "math.h"





/*
typedef struct 
{
  float Des;//控制变量目标值
  float FB;//控制变量反馈值
	
	float Kp;//比例系数Kp
	float Ki;//积分系数Ki
	float Kd;//微分系数Kd
	
	float Up;//比例输出
	float Ui;//积分输出
	float Ud;//微分输出
	
	float E;//本次偏差
	float PreE;//上次偏差
  float SumE;//总偏差
	float U;//本次PID运算结果
	
	float UMax;//PID运算后输出最大值及做遇限削弱时的上限值
	float UpMax;//比例项输出最大值
	float UiMax;//积分项输出最大值
	float UdMax;//微分项输出最大值
	float SumEMax;//积分饱和值
	float EMin;//积分分离阈值
}PIDTypeDef;*/


//void PIDUpdate(PIDTypeDef *pPID);
//void NLPIDUpdate(PIDTypeDef *pPID);


//extern PIDTypeDef RollPID;
//extern PIDTypeDef GyroxPID;

#endif

