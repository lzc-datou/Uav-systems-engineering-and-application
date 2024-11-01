#include "tim4_int.h"
#include "control.h"
#include "tim2.h"

// Vofa+ data protocol
#define 	BYTE0(dwTemp) 	(*((char*)(&dwTemp)))
#define 	BYTE1(dwTemp) 	(*((char*)(&dwTemp)+1))
#define 	BYTE2(dwTemp) 	(*((char*)(&dwTemp)+2))
#define 	BYTE3(dwTemp) 	(*((char*)(&dwTemp)+3))
#define		VOFA_LEN		12

u8 u_Times=0;												// Vofa+ data send interval
char str_USART[VOFA_LEN];						// Vofa+ data array
void Set_USART_Send_String(void);		// Vofa+ data generation

///////////////////////////////////////////////////////////////////
// MAIN control timeline - TIM4 interrupt response function
void TIM4_IRQHandler(void)
{
	int i;
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		TIM2->CR1|=1; TIM2->CNT=0;		// TIM2 counter timer -> for real run time

	// STOP Button on the screen is pushed
		HMI_Check_Button_Push();
		if (Stop_Status == STOP) 
		{
		  M1 = STOP_WIDTH;		
			M2 = STOP_WIDTH;
		}
		HMI_Update();			// Touch screen real-time refresh
		
	// Main handling functions call -> according to current page of touch screen
		switch(CurrentPageId)
		{
			case CONTROL:
			case CURVE:			RotorBalanceControlLoop();			// Auto balance control
											break;
			
			case MOTOR:			Rotor_Motor_Loop();							// MOTOR speed control
											break;
		}		
		
	// Send data to USART2 - VOFA+		
		u_Times ++;
		if (u_Times == 5)
		{
			Set_USART_Send_String();				// Generate Vofa curve data package
			for (i=0; i<VOFA_LEN; i++)			// Vofa bytes USART package
			{
				USART_SendData(USART2, str_USART[i]);
				while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
			}
			u_Times = 0;
		}
		
	// Calculate execution time via TIM2
		tim2cnt[tim2cnt[9]] = TIM2->CNT;
		TIM2->CR1&=(unsigned int)(0xFFFFFFFE);	// Stop TIM2
		if(tim2cnt[9] >= 8) tim2cnt[9] = 0; else tim2cnt[9]++;
	}
}

// TIM4 initialization
void TIM4_Int_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 5000-1;				// 5ms overflow interrupt
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;				// 1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE ); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	PIDInit();		// Set control parameters
	TIM_Cmd(TIM4, ENABLE);		 
}

/////////////////////////////////////////////////////////////////////////////
// Prepare the string to USART
void Set_USART_Send_String(void)
{
	unsigned char _cnt = 0;
	float f_temp;

	str_USART[_cnt++] = BYTE0(roll);
	str_USART[_cnt++] = BYTE1(roll);
	str_USART[_cnt++] = BYTE2(roll);
	str_USART[_cnt++] = BYTE3(roll);
	
	f_temp = Filters.GyroxLPF.output;

	str_USART[_cnt++] = BYTE0(f_temp);
	str_USART[_cnt++] = BYTE1(f_temp);
	str_USART[_cnt++] = BYTE2(f_temp);
	str_USART[_cnt++] = BYTE3(f_temp);	

	str_USART[_cnt++] = 0x00;
	str_USART[_cnt++] = 0x00;
	str_USART[_cnt++] = 0x80;
	str_USART[_cnt++] = 0x7F;
	
}

