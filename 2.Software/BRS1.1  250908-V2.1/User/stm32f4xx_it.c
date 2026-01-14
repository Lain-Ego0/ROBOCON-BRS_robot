#include "stm32f4xx_it.h"
#include "bsp_debug_usart.h"
#include "bsp_general_tim.h"
#include "bsp_debug_usart.h"
#include "pid.h"
#include "GO_M8010_6.h"
#include "motor_control.h" 
#include "bsp_usart_dma.h"
#include "remote.h"

extern uint8_t Motor_Rxflag;
extern uint8_t Motor_Rx_date;
void RS485_USART_IRQHandler(void)
{
}	

extern int step ;
extern int tim_t;
extern int time_currently ;//步态当前时间
extern int time_turn;//转向当前时间

extern	MOTOR_recv motor_feedback_data;     //DMA存储数据后，在中断中用于电机ID判断
extern	MOTOR_send motor_control_clear;     //电机停止

extern	MOTOR_send motor2_control_data;   	//电机2控制
extern	MOTOR_recv motor2_feedback_data;	//电机2反馈
extern	MOTOR_send motor3_control_data;   	//电机3控制
extern	MOTOR_recv motor3_feedback_data;	//电机3反馈
extern	MOTOR_send motor4_control_data;     //电机4控制
extern	MOTOR_recv motor4_feedback_data;	//电机4反馈
extern	MOTOR_send motor5_control_data;     //电机5控制
extern	MOTOR_recv motor5_feedback_data;	//电机5反馈
extern	MOTOR_send motor6_control_data;     //电机6控制
extern	MOTOR_recv motor6_feedback_data;	//电机6反馈
extern	MOTOR_send motor7_control_data;     //电机7控制
extern	MOTOR_recv motor7_feedback_data;	//电机7反馈
extern	MOTOR_send motor8_control_data;     //电机8控制
extern	MOTOR_recv motor8_feedback_data;	//电机8反馈
extern	MOTOR_send motor9_control_data;     //电机8控制
extern	MOTOR_recv motor9_feedback_data;	//电机8反馈

extern int Motor_send_ID;	//2--到--9号电机轮流发控制电机数据

/* 通用定时器2定时，pid ---- 1ms产生一次中断 */

int Motor_feedback_ID = 1;
int tim_flag_aa = 0;
int RC_RX_flag = 0;//DJI遥控接收数据
//狗初始化站立起来时间
int Dog_Iinit = 0;
int Dog_flag = 1;
//跳跃计时
int dog_jump_time = 0;		//大跳越	
int dog_jump_flag = 0;		//大跳越开始计时标志	
int dog_jump_time_2 = 0;	//小跳越	
int dog_jump_flag_2 = 0;	//小跳越开始计时标志	
int dog_jump_time_3 = 0;	//跳远
int dog_jump_flag_3 = 0;	//跳远开始计时标志	
int backflip_flag = 0;		//空翻开始计时标志	
int backflip_time = 0;		//空翻计时
//步幅转向标志
int step_turn_flag = 1;
int step_cycle; //整个运行周期
		
//----1-毫秒中断

void GENERAL_TIM_IRQHandler (void)
{
	if(Dog_Iinit <2001) Dog_Iinit++;//狗初始化站立起来时间
	time_currently++;
	time_turn++;
	if(time_currently >= step_cycle)
	{
		time_currently = 0;
		Dog_flag = 0;
		step_turn_flag = 0;//---步幅转向标志--步幅转向周期----
	}
	if(time_turn >= 400) time_turn = 0;//转向时间
	if(dog_jump_flag)dog_jump_time++;//跳--时间
	if(dog_jump_flag_2)dog_jump_time_2++;//跳--时间
	if(dog_jump_flag_3)dog_jump_time_3++;//跳--时间
    //清除中断标志
	TIM_ClearITPendingBit(GENERAL_TIM , TIM_IT_Update);  		 
}


//电机数据的ID辨别
void Motor_DMA_IRQHandler (void)
{		
	//判断CRC把数据写入接收结构体数据赋值
	extract_data(&motor_feedback_data);
	Motor_feedback_ID = motor_feedback_data.motor_id;
	switch(Motor_feedback_ID)
	{
	case 2 :  
		step = 2;
		motor2_feedback_data.T = motor_feedback_data.T;
		motor2_feedback_data.W = motor_feedback_data.W;
		motor2_feedback_data.Pos = motor_feedback_data.Pos;	
		break;
	case 3 :
		step = 3;			
		motor3_feedback_data.T = motor_feedback_data.T;
		motor3_feedback_data.W = motor_feedback_data.W;
		motor3_feedback_data.Pos = motor_feedback_data.Pos;	
		break;
	case 4 :  
		step = 4;			
		motor4_feedback_data.T = motor_feedback_data.T;
		motor4_feedback_data.W = motor_feedback_data.W;
		motor4_feedback_data.Pos = motor_feedback_data.Pos;
		break;		
	case 5 :  
		step = 5;			
		motor5_feedback_data.T = motor_feedback_data.T;
		motor5_feedback_data.W = motor_feedback_data.W;
		motor5_feedback_data.Pos = motor_feedback_data.Pos;	
		break;
	case 6 :  
		step = 6;			
		motor6_feedback_data.T = motor_feedback_data.T;
		motor6_feedback_data.W = motor_feedback_data.W;
		motor6_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
	case 7 :  
		step = 7;			
		motor7_feedback_data.T = motor_feedback_data.T;
		motor7_feedback_data.W = motor_feedback_data.W;
		motor7_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
	case 8 :  
		step = 8;			
		motor8_feedback_data.T = motor_feedback_data.T;
		motor8_feedback_data.W = motor_feedback_data.W;
		motor8_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
	case 9 :  
		step = 9;			
		motor9_feedback_data.T = motor_feedback_data.T;
		motor9_feedback_data.W = motor_feedback_data.W;
		motor9_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
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



