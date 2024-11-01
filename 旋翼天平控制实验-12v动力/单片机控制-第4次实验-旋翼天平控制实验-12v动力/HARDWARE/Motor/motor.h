#ifndef __MOTOR_H
#define __MOTOR_H	 

#include "stm32f10x_tim.h"
#include "delay.h"

#define 		STOP_WIDTH 			3000			// motor stop
#define 		FULL_WIDTH			6000			// motor full speed
#define 		INIT_WIDTH_1    3800    	// CCR1(0)
#define 		INIT_WIDTH_2    3800      // CCR2(0)
#define			DEAD_BAND				200

#define			MOTOR_KV				2300			// RS2205
#define			MOTOR_VOLTAGE		12				// Power supply
#define			MOTOR_SPEED			270.0			// Pointer range on touch screen

#define			WIDTH_MAX				4500			// Designed motor full speed
#define			WIDTH_MIN				3200			// Designed motor lowest speed

#define			M1							TIM3->CCR3		// Left motor
#define			M2							TIM3->CCR4		// Right motor



void Throttle_Calibration(void);
void Do_Stop_Width(void);
void Soft_Start(void);

	 				    
#endif
