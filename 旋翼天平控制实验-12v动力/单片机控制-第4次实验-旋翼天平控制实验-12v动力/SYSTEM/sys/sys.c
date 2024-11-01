#include "sys.h"
#include "mpu6050.h"
#include "hmi.h"
#include "usart1.h"
#include "usart2.h"
#include "tim2.h"
#include "motor.h"
#include "tim4_int.h"

// Main system initialization function
void Initialization(void)
{
	delay_init();
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	TIM_Configuration();

// MPU6050 initialization
	delay_ms(500);
	if (MPU_Init())
		Set_LED(COLOR_BLU);
	else
		Set_LED(COLOR_GRN);
	
	TASK_timer_TIM2_Init();		// Execution time monitor TIMER
	Usart1_Init();						// HMI touch screen
	Usart2_Init();						// Vofa+ curve monitor
	
	HMI_Print_Init();					// Touch screen first-time display
	Get_Gyro_Static_Error();	// Perform MPU6050 static error acquiring
//	HMI_RefreshPage();

	TIM4_Int_Init(); 					// Main control loop TIMER
}

void NVIC_Configuration(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
}

void RCC_Configuration(void)
{
	SystemInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		// Motor 1,2 PWM channel
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	       
// MOTOR PWM: TIM3 CH3, CH4	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	
// RGB LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	// GREEN
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
 	GPIO_Init(GPIOA, &GPIO_InitStructure);   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;	// 	BLUE|RED
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
 	GPIO_Init(GPIOB, &GPIO_InitStructure);   
}

void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

// MOTOR PWM: TIM3 CH3, CH4
	TIM_TimeBaseStructure.TIM_Period = 10000-1; //300Hz PWM
	TIM_TimeBaseStructure.TIM_Prescaler = 24-1; //3M frequency
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

// TIM3 CH3, CH4 pwm output
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = STOP_WIDTH;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);	// CH3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);	// CH4
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);	
}

// RGB LED manipulation
void Set_LED(u8 color)
{
	switch(color)
	{
		case COLOR_BLK:	PAout(8) = 1; PBout(14) = 1; PBout(15) = 1; break;
		case COLOR_RED:	PAout(8) = 1; PBout(14) = 1; PBout(15) = 0; break;
		case COLOR_GRN:	PAout(8) = 0; PBout(14) = 1; PBout(15) = 1; break;
		case COLOR_BLU:	PAout(8) = 1; PBout(14) = 0; PBout(15) = 1; break;
		case COLOR_WHT:	PAout(8) = 0; PBout(14) = 0; PBout(15) = 0; break;
	}
}
