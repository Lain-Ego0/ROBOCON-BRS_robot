#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "bsp_clkconfig.h"
#include "bsp_debug_usart.h"
#include "GO_M8010_6.h"
#include "bsp_usart_dma.h"
#include "pid.h"
#include "bsp_general_tim.h"
#include "bsp_key.h" 
#include "motor_control.h" 
#include "remote.h"
#include <string.h>

// 系统控制节拍标志 (1ms 触发一次)
extern volatile uint8_t g_system_tick;

#endif


