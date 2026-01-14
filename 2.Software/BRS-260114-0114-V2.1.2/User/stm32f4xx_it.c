#include "stm32f4xx_it.h"
#include "bsp_debug_usart.h"
#include "bsp_general_tim.h"
#include "bsp_debug_usart.h"
#include "pid.h"
#include "GO_M8010_6.h"
#include "motor_control.h" 
#include "bsp_usart_dma.h"
#include "remote.h"
#include "main.h"

/* 外部变量引用 */
extern int step ;
extern int Motor_send_ID;	//2--到--9号电机轮流发控制电机数据
extern int tim_t;
extern int time_currently ;//步态当前时间
extern int time_turn;//转向当前时间
extern int step_turn_flag = 1;//步幅转向标志
extern int step_cycle; //整个运行周期
extern int Dog_Iinit = 0;//狗初始化站立起来时间
extern int Dog_flag = 1;

/* 电机数据结构体引用 */
extern MOTOR_recv motor_feedback_data;
extern MOTOR_recv motor2_feedback_data;
extern MOTOR_recv motor3_feedback_data;
extern MOTOR_recv motor4_feedback_data;
extern MOTOR_recv motor5_feedback_data;
extern MOTOR_recv motor6_feedback_data;
extern MOTOR_recv motor7_feedback_data;
extern MOTOR_recv motor8_feedback_data;
extern MOTOR_recv motor9_feedback_data;


/* -------------------------------------------------------- */
/* 本文件内部变量 */
int Motor_feedback_ID = 1;
int RC_RX_flag = 0;//DJI遥控接收数据

//跳跃计时
extern int dog_jump_time = 0;		//大跳越	
extern int dog_jump_flag = 0;		//大跳越开始计时标志	
extern int dog_jump_time_2 = 0;		//小跳越	
extern int dog_jump_flag_2 = 0;		//小跳越开始计时标志	
extern int dog_jump_time_3 = 0;		//跳远
extern int dog_jump_flag_3 = 0;		//跳远开始计时标志	
		
/* -------------------------------------------------------- */
/* 通用定时器2中断 (1ms) */
/* 负责所有运动学的时间基准 */
void GENERAL_TIM_IRQHandler (void)
{
	// 1. 初始化倒计时
	if(Dog_Iinit <2001) Dog_Iinit++;
	
	// 2. 步态时间更新
	time_currently++;
	if(time_currently >= step_cycle)
	{
		time_currently = 0;
		Dog_flag = 0;
		step_turn_flag = 0;//---步幅转向标志--步幅转向周期----
	}
	// 3. 转向时间更新
	time_turn++;
	if(time_turn >= 400) time_turn = 0;//转向时间
	
	// 4. 跳跃计时更新
	if(dog_jump_flag)dog_jump_time++;
	if(dog_jump_flag_2)dog_jump_time_2++;
	if(dog_jump_flag_3)dog_jump_time_3++;
	
    //清除中断标志
	TIM_ClearITPendingBit(GENERAL_TIM , TIM_IT_Update);  		 
}


/* -------------------------------------------------------- */
/* DMA数据接收中断 */
/* 负责解析电机反馈并触发主循环计算 */
void Motor_DMA_IRQHandler (void)
{		
	// 解析数据
	extract_data(&motor_feedback_data);
	Motor_feedback_ID = motor_feedback_data.motor_id;
	
	// 根据ID分发数据
	switch(Motor_feedback_ID)
	{
	case 2 : step = 2; motor2_feedback_data = motor_feedback_data; break; // 结构体直接赋值
	case 3 : step = 3; motor3_feedback_data = motor_feedback_data; break;
	case 4 : step = 4; motor4_feedback_data = motor_feedback_data; break;		
	case 5 : step = 5; motor5_feedback_data = motor_feedback_data; break;
	case 6 : step = 6; motor6_feedback_data = motor_feedback_data; break;
	case 7 : step = 7; motor7_feedback_data = motor_feedback_data; break;
	case 8 : step = 8; motor8_feedback_data = motor_feedback_data; break;
	case 9 : step = 9; motor9_feedback_data = motor_feedback_data; break;
	}
	
	DMA_ClearITPendingBit(DEBUG_USART_DMA_STREAM , DMA_IT_TCIF1); 
}

void RC_DMA_IRQHandler(void)
{
	RC_RX_flag = 1;
	DMA_ClearITPendingBit(RC_USART_DMA_STREAM , DMA_IT_TCIF2); 
}

void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



