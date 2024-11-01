#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "stm32f10x_tim.h"
#include <math.h>
#include <stdio.h>
#include "mpu6050.h"
#include "motor.h"
#include "IMU.h"
#include "hmi.h"
#include "filter.h"

// Calculation macros
#define abs(x) ( (x)>0?(x):-(x) )
#define value_limit(x,small,big)   if(x<small)x=small;if(x>big)x=big;

// Control interval
#define CONTROL_INTER		0.005
#define ANGLE_RANGE			20

// Touch screen macros
#define STOP 1
#define RUN  0

#define CONTROL	0
#define CURVE		1
#define MOTOR		2


extern unsigned char Stop_Status;

// Motor driven variables
extern int n_L_Motor_Width, n_R_Motor_Width;

// IMU variables
extern float pitch, roll, yaw, gyroGx, gyroGy, gyroGz;
extern float scope_angle, scope_gyro;
extern float Angle_Des, Angle_Des_Last;							// Angle expectation


//////////////////////////////////////////////////////////////////////
// Control variants
extern float Kp_OuterLoop, Ki_OuterLoop, Kd_OuterLoop;
extern float Kp_InnerLoop, Ki_InnerLoop, Kd_InnerLoop;
extern float Angle_Rad, Angular_Velocity_Rad;
extern float Angle_Rad_Last;
extern float Angle_Rad_Acc;
extern float Output_OuterLoop;
extern float Angular_Velocity_Rad_Acc;
extern float Angular_Velocity_Rad_Last;
extern float Output_OuterLoop_Last;			
extern float f_Lambda;
extern float f_Control_Limit;


//////////////////////////////////////////////////////////////////////
void PIDInit(void);
void RotorBalanceControlLoop(void);
void Rotor_Motor_Loop(void);
void LPFUpdate6axis(void);



#endif
