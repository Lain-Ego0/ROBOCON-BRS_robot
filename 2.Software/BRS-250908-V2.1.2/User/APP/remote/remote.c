#include "remote.h"
#include "bsp_usart_dma.h"

//遥控数据-----------------------------------------------
extern uint8_t RC[18];
rc rc_rc;
//-------------------------------------------------------
void rc_rc_date(void)
{
	rc_rc.ch0 = 0;
	rc_rc.ch1 = 0;
	rc_rc.ch2 = 0;
	rc_rc.ch3 = 0;
	rc_rc.s1 = 1;//值---1，2，3
	rc_rc.s2 = 1;//值---1，2，3
	//注意数据的顺序----
	rc_rc.ch0 = ((int16_t)RC[0] | ((int16_t)RC[1] << 8)) & 0x07FF; 
	rc_rc.ch1 = (((int16_t)RC[1] >> 3) | ((int16_t)RC[2] << 5))& 0x07FF;
	rc_rc.ch2 = (((int16_t)RC[2] >> 6) | ((int16_t)RC[3] << 2) |((int16_t)RC[4] << 10)) & 0x07FF;
	rc_rc.ch3 = (((int16_t)RC[4] >> 1) | ((int16_t)RC[5]<<7))&0x07FF;
	//遥控数(max660)  乘  系数 = 》给电机数值   转速max = 
	if(0 == ((rc_rc.ch0 > 363)&&(rc_rc.ch1 > 363)&&(rc_rc.ch2 > 363)&&(rc_rc.ch3 > 363)&&\
	(rc_rc.ch0 < 1685)&&(rc_rc.ch1 < 1685)&&(rc_rc.ch2 < 1685)&&(rc_rc.ch3 < 1685)))
	{
		rc_rc.ch0 = 0;
		rc_rc.ch1 = 0;
		rc_rc.ch2 = 0;
		rc_rc.ch3 = 0;
	}
	else
	{
		rc_rc.ch0 = (rc_rc.ch0 - 1024);
		rc_rc.ch1 = (rc_rc.ch1 - 1024);
		rc_rc.ch2 = (rc_rc.ch2 - 1024);
		rc_rc.ch3 = (rc_rc.ch3 - 1024);
	}
		rc_rc.s1 = ((RC[5] >> 4) & 0x000C) >> 2;
		rc_rc.s2 = ((RC[5] >> 4) & 0x0003);
}


