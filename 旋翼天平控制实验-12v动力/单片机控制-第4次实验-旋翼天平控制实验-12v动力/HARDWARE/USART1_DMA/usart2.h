#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"	
#include "stm32f10x_usart.h" 
#include "stm32f10x_dma.h"

// Vofa+ USART
#define USART2_BAUD      115200
#define USART2_BUF_LEN   500  

#define USART2_DMA_RX_CHANNEL   DMA1_Channel6
#define USART2_DMA_TX_CHANNEL   DMA1_Channel7

extern u8  		USART2_RX_BUF[USART2_BUF_LEN];
extern u8			USART2_TX_BUF[USART2_BUF_LEN]; 

void Usart2_Init(void);
void USART2_DMA_TX_Enable(unsigned int buf_bytes);

#define DMA_RX_EN(DMA_CH,LEN);          DMA_Cmd(DMA_CH, DISABLE );\
																					DMA_CH->CNDTR =LEN;\
																					DMA_Cmd(DMA_CH, ENABLE);
																					
#endif
