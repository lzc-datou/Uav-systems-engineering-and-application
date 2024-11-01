#include "usart2.h"	  
#include "hmi.h"

// Vofa+ USART -> DMA
u8	USART2_RX_BUF[USART2_BUF_LEN]; 
u8	USART2_TX_BUF[USART2_BUF_LEN]; 
unsigned int USART2_REC_CNT;	

// RX data handling
void USART2_IRQHandler(void)
{	
	unsigned char reg;
  if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
  {
		reg=USART2->SR;	
		reg=USART2->DR;
		reg=reg;
		USART2_REC_CNT = USART2_BUF_LEN - USART2_DMA_RX_CHANNEL->CNDTR;
		DMA_RX_EN(USART2_DMA_RX_CHANNEL,USART2_BUF_LEN);
	}
}

void USART2_DMA_TX_Enable(unsigned int buf_bytes) 
{ 
	DMA_Cmd(USART2_DMA_TX_CHANNEL, DISABLE );   
	USART2_DMA_TX_CHANNEL->CNDTR = buf_bytes;
 	DMA_Cmd(USART2_DMA_TX_CHANNEL, ENABLE);
}

void Usart2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_DeInit(USART2);
	
// USART2_TX	  PA2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
// USART2_RX	  PA3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// Usart2 NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
// USART2 init
	USART_InitStructure.USART_BaudRate = USART2_BAUD;		//115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART2, ENABLE);
}


