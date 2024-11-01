#ifndef __USART1_H
#define __USART1_H
#include "stdio.h"	
#include "stm32f10x_usart.h" 
#include "stm32f10x_dma.h"

#define USART1_BAUD    			  512000		// Touch screen high rate
#define USART1_BUF_LEN  			500

#define USART1_DMA_RX_CHANNEL   DMA1_Channel5
#define USART1_DMA_TX_CHANNEL   DMA1_Channel4

extern u8  		USART1_RX_BUF[USART1_BUF_LEN];
extern u8			USART1_TX_BUF[USART1_BUF_LEN]; 

void Usart1_Init(void);
void USART1_DMA_TX_Enable(unsigned int buf_bytes);

#define DMA_RX_EN(DMA_CH,LEN);          DMA_Cmd(DMA_CH, DISABLE );\
																					DMA_CH->CNDTR =LEN;\
																					DMA_Cmd(DMA_CH, ENABLE);
																					
#endif
