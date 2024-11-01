#ifndef __TIM2_H
#define __TIM2_H
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

void TASK_timer_TIM2_Init(void);
 
extern unsigned short tim2cnt[10];

#endif
