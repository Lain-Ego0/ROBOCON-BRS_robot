#include "stm32f4xx_it.h"
#include "bsp_debug_usart.h"
#include "bsp_general_tim.h"
#include "pid.h"
#include "GO_M8010_6.h"
#include "motor_control.h" 
#include "bsp_usart_dma.h"
#include "remote.h"

extern Motor_Unit_TypeDef Leg_Motors[8];
extern volatile uint8_t g_system_tick;
extern MOTOR_recv motor_feedback_data; 

// 变量引用 (Dog_Iinit 已移交 motor_control.c 管理，此处仅为了兼容 extern 定义，不进行操作)
extern int Dog_Iinit;
int Dog_flag = 1;

// 跳跃变量 (计数逻辑保留在中断中，或者也可以移出，但暂时保留以防影响时序)
extern int dog_jump_time;		
extern int dog_jump_flag;		
extern int dog_jump_time_2;	
extern int dog_jump_flag_2;	
extern int dog_jump_time_3;	
extern int dog_jump_flag_3;	

int step_turn_flag = 1;
extern int step_cycle; 
extern int time_currently;
extern int time_turn;

// -----------------------------------------------------------
// 1ms 定时器中断
// -----------------------------------------------------------
void GENERAL_TIM_IRQHandler (void)
{
    g_system_tick = 1;

    // [删除] Dog_Iinit 的自增逻辑已移至 motor_control.c 的 Handle_System 中
    // if(Dog_Iinit < 2001) Dog_Iinit++; 
	
    time_currently++;
	time_turn++;
	
	if(time_currently >= step_cycle)
	{
		time_currently = 0;
		Dog_flag = 0;
		step_turn_flag = 0;
	}
	
	if(time_turn >= 400) time_turn = 0;
	
    // 跳跃计时
	if(dog_jump_flag) dog_jump_time++;
	if(dog_jump_flag_2) dog_jump_time_2++;
	if(dog_jump_flag_3) dog_jump_time_3++;
    
	TIM_ClearITPendingBit(GENERAL_TIM , TIM_IT_Update);  		 
}

// -----------------------------------------------------------
// 电机 DMA 接收中断
// -----------------------------------------------------------
void Motor_DMA_IRQHandler (void)
{		
	extract_data(&motor_feedback_data);
    int id = motor_feedback_data.motor_id;
    if (id >= 2 && id <= 9) {
        int index = id - 2; 
        Leg_Motors[index].feedback.T = motor_feedback_data.T;
        Leg_Motors[index].feedback.W = motor_feedback_data.W;
        Leg_Motors[index].feedback.Pos = motor_feedback_data.Pos;
    }
	DMA_ClearITPendingBit(DEBUG_USART_DMA_STREAM , DMA_IT_TCIF1); 
}

void RC_DMA_IRQHandler(void)
{
    extern volatile int RC_RX_flag; 
	RC_RX_flag = 1;
	DMA_ClearITPendingBit(RC_USART_DMA_STREAM , DMA_IT_TCIF2); 
}

void NMI_Handler(void){}
void HardFault_Handler(void){while(1){}}
void MemManage_Handler(void){while(1){}}
void BusFault_Handler(void){while(1){}}
void UsageFault_Handler(void){while(1){}}
void SVC_Handler(void){}
void DebugMon_Handler(void){}
void PendSV_Handler(void){}
void SysTick_Handler(void){}
