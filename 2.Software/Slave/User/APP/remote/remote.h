#ifndef __USART3_DMA_H
#define	__USART3_DMA_H
#include "stm32f4xx.h"
typedef struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t s1;
	uint8_t s2;
}rc;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}mouse;
typedef struct
{
	uint16_t v;
}key;
extern rc rc_rc;
void rc_rc_date(void);
#endif /* __USART_dma */
