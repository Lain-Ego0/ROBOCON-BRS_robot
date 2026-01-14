#ifndef __USART_DMA_H
#define	__USART_DMA_H

#include "stm32f4xx.h"
#include <stdio.h>

//DMA----电机
#define DEBUG_USART_DR_BASE             (USART6_BASE+0x04)		
#define SENDBUFF_SIZE                   16	//发送的数据量
#define DEBUG_USART_DMA_CLK               RCC_AHB1Periph_DMA2
#define DEBUG_USART_DMA_CHANNEL           DMA_Channel_5
#define DEBUG_USART_DMA_STREAM            DMA2_Stream1
//---中断
#define Motor_DMA_IRQn                      DMA2_Stream1_IRQn 
#define Motor_DMA_IRQHandler                DMA2_Stream1_IRQHandler
//DMA______RC
#define RC_USART_DR_BASE               (USART1_BASE+0x04)		
#define RC_SENDBUFF_SIZE                18	//发送的数据量
#define RC_USART_DMA_CLK               RCC_AHB1Periph_DMA2
#define RC_USART_DMA_CHANNEL           DMA_Channel_4
#define RC_USART_DMA_STREAM            DMA2_Stream2
//DMA_RC中断
#define RC_USART1_IRQn									DMA2_Stream2_IRQn
#define RC_DMA_IRQHandler           		DMA2_Stream2_IRQHandler

void USART6_DMA_Config(void);
void USART1_DMA_Config(void);
void Motor_DMA_NVIC_Configuration(void);
void RC_DMA_NVIC_Configuration(void);
#endif 
