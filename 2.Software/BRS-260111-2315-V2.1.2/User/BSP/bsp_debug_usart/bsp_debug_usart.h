#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

//-------------------------------------------------------------//
//中断函数名
#define RS485_USART_IRQHandler                   USART6_IRQHandler
#define RS485_USART_IRQ                 					USART6_IRQn
/************************************************************/
//中断函数名
#define RC_USART_IRQHandler                   USART1_IRQHandler
#define RC_USART_IRQ                 					USART1_IRQn

void NVIC_Configuration(void);
void HOST_USART_Config(void);
void MOTOR_CONTROL1_USART_Config(void);
void RC_USART1_Config(void);
	
static void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch );
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
void Usart_SendString( USART_TypeDef * pUSARTx, uint8_t *str);
//int fputc(int ch, FILE *f);

#endif /* __USART1_H */
