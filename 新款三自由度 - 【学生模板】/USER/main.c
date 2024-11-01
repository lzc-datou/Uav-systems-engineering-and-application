#include "stm32f10x.h"    
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "delay.h"
#include "control.h"
#include "usart.h"
#include "mpu6050.h"
#include "filter.h"

#define 	BYTE0(dwTemp) 	(*((char*)(&dwTemp)))
#define 	BYTE1(dwTemp) 	(*((char*)(&dwTemp)+1))
#define 	BYTE2(dwTemp) 	(*((char*)(&dwTemp)+2))
#define 	BYTE3(dwTemp) 	(*((char*)(&dwTemp)+3))
#define 	VOFA_LEN	16

#define COLOR_WHT		0
#define COLOR_RED 	1
#define COLOR_GRN		2
#define COLOR_BLU		3

//////////////////////////////////////////////////////////////////////////////////
char str_USART[VOFA_LEN];

short ax, ay, az, gx, gy, gz;
float gyroGx, gyroGy, gyroGz;
 
u8 b_Lock=1;
u8 u_Times=0;
 
//////////////////////////////////////////////////////////////////////////////////

Filter6axisTypeDef Filters;
 
/////////////////////////////////////////////////
////////////////  FUNCTIONS  ////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
void RCC_Configuration(void);
void GPIO_Configuration(void);
void Get_Gyro_Static_Error(void);
void TIM1_Int_Init(void);
void TIM3_PWM_4CH_Init(void);
void Stop_Motors(void);
void Set_USART_Send_String(void);
void Set_LED(char color);

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

//x:roll  right-		y:pitch		forward-		z:yaw		clockwise-

/***********

        M3  (CW)                   M2 (CCW)
				  \                        /
				    \                     /
				      \                 /
				          
				         /            \
				        /              \
				       /                 \
				      /                   \
				    /                      \
				M4 (CCW)                   M1 (CW)
                 
***********/


int main(void)
{ 
	delay_init();
	RCC_Configuration();  
	GPIO_Configuration();
	Set_LED(COLOR_RED);	
	
// Motor INITIALIZATION
	TIM3_PWM_4CH_Init();
	Stop_Motors();
	
	PID_Init();
	uart_init(115200);
	
	delay_ms(800);
	if (MPU_Init())
		Set_LED(COLOR_BLU);
	delay_ms(200);
	Get_Gyro_Static_Error();
	
	TIM1_Int_Init();
	Set_LED(COLOR_GRN);
	
	while(1)
	{   
		
	}
}


///////////////////////////////////////////////////////////////////////////
////////////////// MAIN CONTROL TIMER 5ms /////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void TIM1_UP_IRQHandler(void)  
{
	int i;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  		// every 5ms
		
	// Get MPU6050 raw data
		MPU_Get_Accelerometer(&ax, &ay, &az);
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		
	// Static error compensation
		gyroGx = (gx - f_gx_error)/16.384;
		gyroGy = (gy - f_gy_error)/16.384;
		gyroGz = (gz - f_gz_error)/16.384;
	
	// Low pass filtering & Mahony algorithm
		LPFUpdate6axis();
		IMUupdate(Filters.GyroxFinal.output/57.3,
							Filters.GyroyFinal.output/57.3,
							Filters.GyrozFinal.output/57.3,
							Filters.AccxLPF.output,
							Filters.AccyLPF.output,
							Filters.AcczLPF.output,
							&pitch, &roll, &yaw);
	
	////////////////////////////////////////////////////////////////
	// b_Lock is the variant to stop motors & control law
		if (b_Lock)
		{
			PBout(15) = 0;
			Stop_Motors();
		}
		
	////////////////////////////////////////////////////////////////
	// b_Lock is released. Control law works
		else
		{
			PBout(15) = 1;
			PID_Control(Filters.GyroxFinal.output/57.3,
									Filters.GyroyFinal.output/57.3,
									Filters.GyrozFinal.output/57.3);
		}
		
		// Send Euler angles to USART1		
			u_Times ++;
			if (u_Times == 5)
			{
				Set_USART_Send_String();	// Generate Vofa curve data package
				for (i=0; i<VOFA_LEN; i++)			// Vofa 16 bytes USART package
				{
					USART_SendData(USART1, str_USART[i]);
					while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
				}
				u_Times = 0;
			}
		
	}
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////// INITIALIZATION FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void RCC_Configuration(void)
{
	SystemInit();   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
}

void GPIO_Configuration(void)		// LEDs
{
	GPIO_InitTypeDef GPIO_InitStructure;   
	       
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);   
}

///////////////////////////////////////////////////////////////////////////////////
//////////////////////// CONTROL TIMER 5ms  ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void TIM1_Int_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = 5000-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	TIM1->RCR = 0;
	TIM_Cmd(TIM1, ENABLE); 
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// MOTOR DRIVE TIM INIT //////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void TIM3_PWM_4CH_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 10000-1;      
  TIM_TimeBaseStructure.TIM_Prescaler = 24-1;	  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

void Stop_Motors(void)
{
	M1 = Motor_PWM_MIN;
	M2 = Motor_PWM_MIN;
	M3 = Motor_PWM_MIN;
	M4 = Motor_PWM_MIN;
}

/////////////////////////////////////////////////////////////////////////////
// Prepare the string to USART
void Set_USART_Send_String(void)
{
	unsigned char _cnt = 0;

	str_USART[_cnt++] = BYTE0(pitch);
	str_USART[_cnt++] = BYTE1(pitch);
	str_USART[_cnt++] = BYTE2(pitch);
	str_USART[_cnt++] = BYTE3(pitch);

	str_USART[_cnt++] = BYTE0(roll);
	str_USART[_cnt++] = BYTE1(roll);
	str_USART[_cnt++] = BYTE2(roll);
	str_USART[_cnt++] = BYTE3(roll);
	
	str_USART[_cnt++] = BYTE0(yaw);
	str_USART[_cnt++] = BYTE1(yaw);
	str_USART[_cnt++] = BYTE2(yaw);
	str_USART[_cnt++] = BYTE3(yaw);
/*	
	str_USART[_cnt++] = BYTE0(ax);
	str_USART[_cnt++] = BYTE1(ax);
	str_USART[_cnt++] = BYTE2(ax);
	str_USART[_cnt++] = BYTE3(ax);	
	
	str_USART[_cnt++] = BYTE0(ay);
	str_USART[_cnt++] = BYTE1(ay);
	str_USART[_cnt++] = BYTE2(ay);
	str_USART[_cnt++] = BYTE3(ay);		
	
	str_USART[_cnt++] = BYTE0(az);
	str_USART[_cnt++] = BYTE1(az);
	str_USART[_cnt++] = BYTE2(az);
	str_USART[_cnt++] = BYTE3(az);		
	
	str_USART[_cnt++] = BYTE0(gx);
	str_USART[_cnt++] = BYTE1(gx);
	str_USART[_cnt++] = BYTE2(gx);
	str_USART[_cnt++] = BYTE3(gx);		
	
	str_USART[_cnt++] = BYTE0(gy);
	str_USART[_cnt++] = BYTE1(gy);
	str_USART[_cnt++] = BYTE2(gy);
	str_USART[_cnt++] = BYTE3(gy);		
	
	str_USART[_cnt++] = BYTE0(gz);
	str_USART[_cnt++] = BYTE1(gz);
	str_USART[_cnt++] = BYTE2(gz);
	str_USART[_cnt++] = BYTE3(gz);		*/
	
	str_USART[_cnt++] = 0x00;
	str_USART[_cnt++] = 0x00;
	str_USART[_cnt++] = 0x80;
	str_USART[_cnt++] = 0x7F;
	
}

void Set_LED(char color)
{
	switch (color)
	{
		case COLOR_RED:	PAout(8) = 1; PBout(14) = 1; PBout(15) = 0; break;
		case COLOR_GRN:	PAout(8) = 0; PBout(14) = 1; PBout(15) = 1; break;
		case COLOR_BLU:	PAout(8) = 1; PBout(14) = 0; PBout(15) = 1; break;
		case COLOR_WHT:	PAout(8) = 0; PBout(14) = 0; PBout(15) = 0; break;
		default: break;
	}
}

