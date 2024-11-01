#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "stm32f10x_tim.h"
#include <math.h>
#include <stdio.h>


#define abs(x) ( (x)>0?(x):-(x) )
#define value_limit(x,small,big)   if(x<small)x=small;if(x>big)x=big;

#define CONTROL_INTER		0.005

#define Motor_PWM_MIN		3000
#define Motor_PWM_MAX		6000
#define Motor_PWM_IDLE	200

#define M1		TIM3->CCR1
#define M2		TIM3->CCR2
#define M3		TIM3->CCR3
#define M4		TIM3->CCR4

//////////////////////////////////////////////////////////////////////
void PID_Init(void);
void PID_Control(float filter_gyro_x, float filter_gyro_y, float filter_gyro_z);
void PID_Limit(void);

#endif
