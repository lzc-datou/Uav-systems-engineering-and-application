#include "tim2.h"

unsigned short tim2cnt[10];

void TASK_timer_TIM2_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65535; 
	TIM_TimeBaseStructure.TIM_Prescaler =72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
	TIM_Cmd(TIM2, ENABLE);		 
}
