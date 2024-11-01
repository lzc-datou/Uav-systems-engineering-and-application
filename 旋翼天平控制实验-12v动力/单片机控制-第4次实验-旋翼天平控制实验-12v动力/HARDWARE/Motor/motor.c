#include "motor.h"

void Throttle_Calibration(void)
{

}

void Do_Stop_Width(void)
{
	M1 = STOP_WIDTH; 
	M2 = STOP_WIDTH;
}

void Soft_Start(void)
{
	int i,j;
	i = STOP_WIDTH + DEAD_BAND;
	j = STOP_WIDTH + DEAD_BAND;
	
	delay_ms(3000);
	
	while(i<INIT_WIDTH_1 || j<INIT_WIDTH_2)
	{
		if (i<INIT_WIDTH_1)
		{
			i++;
			TIM3->CCR3 = i; 
		}
		if (j<INIT_WIDTH_2)
		{
			j++;
			TIM3->CCR4 = j;
		}
		delay_ms(3);
	}
	
}

