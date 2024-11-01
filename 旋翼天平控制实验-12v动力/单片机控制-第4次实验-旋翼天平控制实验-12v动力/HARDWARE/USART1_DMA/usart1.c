#include "usart1.h"	  
#include "hmi.h"

// HMI touch screen USART
u8	USART1_RX_BUF[USART1_BUF_LEN];
u8	USART1_TX_BUF[USART1_BUF_LEN]; 
unsigned int USART1_REC_CNT;	

// RX data handling
void USART1_IRQHandler(void)
{	
	unsigned char reg;
  if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
  {
		reg=USART1->SR;
		reg=USART1->DR;
		reg=reg;
		USART1_REC_CNT = USART1_BUF_LEN - USART1_DMA_RX_CHANNEL->CNDTR;
		HMI_RX_Deal();		// HMI touch screen data protocol retrieve
		DMA_RX_EN(USART1_DMA_RX_CHANNEL,USART1_BUF_LEN);
	}
}

void USART1_DMA_TX_Enable(unsigned int buf_bytes) 
{ 
	DMA_Cmd(USART1_DMA_TX_CHANNEL, DISABLE);
	USART1_DMA_TX_CHANNEL->CNDTR = buf_bytes;
 	DMA_Cmd(USART1_DMA_TX_CHANNEL, ENABLE);
}

// USART1 init
void Usart1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	USART_DeInit(USART1);
	
// USART1_TX   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
// USART1_RX	  PA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// Usart1 NVIC init
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0; // Touch screen No.1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
// USART init
	USART_InitStructure.USART_BaudRate = USART1_BAUD;		// 512000
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1, ENABLE);

// DMA init
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_RX_BUF;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = USART1_BUF_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel5, ENABLE);

  DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_TX_BUF;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART1_DMA_TX_Enable(1);
}

