#include "main.h"

volatile uint8_t g_system_tick = 0;

/* 全局变量 */
MOTOR_send motor_control_clear; 
uint8_t Motor_Rxflag = 0;
uint8_t Motor_Rx_date = 0;

volatile int RC_RX_flag = 0;

int rc_watchdog_count = 0;
#define RC_TIMEOUT_MAX 500 // 500ms 超时

// 时间与步态参数
int tim_t = 0; // 启动延时
int time_currently = 0;
int time_turn = 0;
int step_rate = 250; 

extern uint8_t RC[18];
extern int Dog_Iinit;
extern int step_turn_flag;

// 发送索引
int motor_send_index = 0; 

// 足端轨迹坐标
float foot_track_x1 = 0.0f, foot_track_y1 = 0.0f;
float foot_track_x2 = 0.0f, foot_track_y2 = 0.0f;
float foot_track_x3 = 0.0f, foot_track_y3 = 0.0f;
float foot_track_x4 = 0.0f, foot_track_y4 = 0.0f;	

int main(void)
{
    // 1. 硬件外设初始化
	RC_USART1_Config();				
	HOST_USART_Config();			
	MOTOR_CONTROL1_USART_Config();	
	
	USART1_DMA_Config();			
	USART6_DMA_Config();			
	
	RC_DMA_NVIC_Configuration();		
	Motor_DMA_NVIC_Configuration();	
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);	
	
	HSE_SetSysClock(12, 336, 2, 7); 

    // 2. 算法模块初始化
	PID_Init();						
	Motor_Mode_Init();				

    // 3. 启动定时器 
	TIMx_Configuration(); 
	
	// 4.上电保护
	while(tim_t < 2000) { 
        if(g_system_tick) { 
            tim_t++; 
            g_system_tick = 0; 
        }
    }
	memset(&rc_rc, 0, sizeof(rc_rc));
    rc_rc.s1 = 2; // 系统模式
    rc_rc.s2 = 3; // 趴下状态
	
	while(1)
	{	
        if (g_system_tick)
        {
            g_system_tick = 0; // 清除标志

            // --- 1. 数据处理 ---
			if (RC_RX_flag == 1) {
                RC_RX_flag = 0;
                rc_rc_date(); // 解析数据
                rc_watchdog_count = 0; // 喂狗
            } else {
                rc_watchdog_count++;
            }
			
			if (rc_watchdog_count > RC_TIMEOUT_MAX) {
                // 遥控器超时强制进入安全模式
                rc_rc.ch0 = 0; rc_rc.ch1 = 0; rc_rc.ch2 = 0; rc_rc.ch3 = 0;
                rc_rc.s1 = 2; 
                rc_rc.s2 = 3; 
                if(rc_watchdog_count > 10000) rc_watchdog_count = 10000; 
            }

            // --- 2. 步态轨迹规划 ---
            foot_track(&foot_track_x1, &foot_track_y1, step_rate, time_currently, 200, time_turn, 2, 1);           
            foot_track(&foot_track_x2, &foot_track_y2, step_rate, time_currently, 200, time_turn, 1, 3);         
            foot_track(&foot_track_x3, &foot_track_y3, step_rate, time_currently, 200, time_turn, 2, 2);          
            foot_track(&foot_track_x4, &foot_track_y4, step_rate, time_currently, 200, time_turn, 1, 4);

            // --- 3. 运动逆解 ---
            counter_motion(foot_track_x1, foot_track_y1, &Leg_Motors[0].target_angle, &Leg_Motors[1].target_angle);
            counter_motion(foot_track_x2, foot_track_y2, &Leg_Motors[3].target_angle, &Leg_Motors[2].target_angle);
            counter_motion(foot_track_x3, foot_track_y3, &Leg_Motors[4].target_angle, &Leg_Motors[5].target_angle);
            counter_motion(foot_track_x4, foot_track_y4, &Leg_Motors[7].target_angle, &Leg_Motors[6].target_angle);

            // --- 4. 控制 ---
            Motor_pid_compute_all();	//PID输出		           
            Motor_data_update_all();	//更新数据帧
            
            // --- 5. 通信发送 --- 大约使用0.8ms，也可以修改运行时间
            for(int k=0; k<8; k++) {
                Motor_date_send_sequential(&motor_send_index);
            }
            
            motor_send_index = 0; // 复位索引确保同步
        }
	}
}

