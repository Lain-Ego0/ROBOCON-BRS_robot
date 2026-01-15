/**
  ******************************************************************************
  * @file    bsp_general_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   通用定时器定时范例
  */
  
#include "bsp_general_tim.h"

 /**
  * @brief  通用定时器 TIMx,x[1,8]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
	// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM_IRQn; 	
	// 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	// 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// 开启TIMx_CLK,x[6,7] 
	RCC_APB1PeriphClockCmd(GENERAL_TIM_CLK, ENABLE); 
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到9，即为10次，为一个定时周期
	TIM_TimeBaseStructure.TIM_Period =100;       
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 840-1;	
	// 采样时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	// 计数方式
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	// 初始化定时器TIMx, x[1,8]
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);
	// 清除定时器更新中断标志位
	TIM_ClearFlag(GENERAL_TIM, TIM_FLAG_Update);
	// 开启定时器更新中断
	TIM_ITConfig(GENERAL_TIM,TIM_IT_Update,ENABLE);
	// 使能定时器
	TIM_Cmd(GENERAL_TIM, ENABLE);	
}

/**
  * @brief  初始化高级控制定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
	TIM_Mode_Config();
}

/*********************************************END OF FILE**********************/
