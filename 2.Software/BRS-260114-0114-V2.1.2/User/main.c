//使用的电机为23，45，67，89
//串口1接大疆遥控
//串口3用于debug（可以不用，串口打印有时候识别不到，不如debug）
//串口6转485控制电机

#include "main.h"


/* 全局变量定义 */
// -----------------------------------------------------------
// 核心控制变量 
int step_rate = 250;      // 半周期时长 (默认值)
int step_cycle = 500;     // 全周期时长 (默认值)
int time_currently = 0;   // 当前行走时间
int time_turn = 0;        // 当前转向时间


int tim_t = 1;////启动延时等待误差结束

extern uint8_t RC[18];

extern int Motor_send_ID;	//2--到--9号电机轮流发控制电机数据
extern int Motor_feedback_ID ;//反馈ID

// 关节角度计算结果
float foot_track_x1 = 0.0f;float foot_track_y1 = 0.0f;
float foot_track_x2 = 0.0f;float foot_track_y2 = 0.0f;
float foot_track_x3 = 0.0f;float foot_track_y3 = 0.0f;
float foot_track_x4 = 0.0f;float foot_track_y4 = 0.0f;	

// 轨迹坐标
float rang__2 = 0.0f;float rang__3 = 0.0f;
float rang__4 = 0.0f;float rang__5 = 0.0f;
float rang__6 = 0.0f;float rang__7 = 0.0f;
float rang__8 = 0.0f;float rang__9 = 0.0f;

// 标志位
int nnn = 0;              // 延时计数
int step_turn_flag = 1;   // 步幅转向标志
int Motor_send_ID = 2;    // 发送ID计数
// -----------------------------------------------------------

extern int step;          // 来自中断的电机回复标志
extern int Dog_Iinit;

int main(void)
{
	RC_USART1_Config();				/*初始化USART1(大疆遥控)*/		
	HOST_USART_Config();			/*初始化USART3（打印电机参数）*/
	MOTOR_CONTROL1_USART_Config();	/*初始化USART6（usart转485）*/
	
	USART1_DMA_Config();			/*USART1的DMA*/
	USART6_DMA_Config();			/*USART6的DMA*/
	
	RC_DMA_NVIC_Configuration();	/*USART1--RC--的DMA接收中断*/	
	Motor_DMA_NVIC_Configuration();	/*USART6的DMA电机数据接收中断中断*/	
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	/* USART1 向 DMA发出RX请求 */	
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);	/* USART6 向 DMA发出RX请求 */
	
	HSE_SetSysClock(12,336, 2, 7);	//系统时钟
	
	Key_GPIO_Config();				/*初始化按键*/	
	PID_Init();						/*PID初始化与限幅*/
	Motor_Mode_Init();				/*给电机初始化*/

	while(tim_t<100000){tim_t++;}/* 初始化通用定时器2定时，1ms产生一次中断 */
	
	TIMx_Configuration();

	while(1)
	{	
		rc_rc_date();//遥控数据更新
		if(step)
		{
			// --- 1号腿 (ID 2,3) & 4号腿 (ID 8,9) ---
			// 设为 MoveMode=2 (相位偏移，形成对角步态)
			if((step == 2)||(step == 3))
			{		
				// 参数：X, Y, 半周期, 当前时间, 转向周期, 转向时间, 模式(2=先摆), 腿方向ID(1)
				foot_track(&foot_track_x1, &foot_track_y1, step_rate, time_currently, step_rate, time_turn, 2, 1);
				counter_motion(foot_track_x1, foot_track_y1, &rang__2, &rang__3);
			}
			else if((step == 8)||(step == 9))
			{
				foot_track(&foot_track_x4, &foot_track_y4, step_rate, time_currently, step_rate, time_turn, 2, 4);
				counter_motion(foot_track_x4, foot_track_y4, &rang__9, &rang__8);
			}
			
			// --- 2号腿 (ID 4,5) & 3号腿 (ID 6,7) ---
			// 设为 MoveMode=1 (相位正常)
			else if((step == 4)||(step == 5))
			{
				foot_track(&foot_track_x2, &foot_track_y2, step_rate, time_currently, step_rate, time_turn, 1, 3);
				counter_motion(foot_track_x2, foot_track_y2, &rang__5, &rang__4);
			}
			else if((step == 6)||(step == 7))
			{
				foot_track(&foot_track_x3, &foot_track_y3, step_rate, time_currently, step_rate, time_turn, 1, 2);
				counter_motion(foot_track_x3, foot_track_y3, &rang__6, &rang__7);
			}
			
			Motor_pid_count(step,rc_rc.ch0*0.015f);			
			Motor_data_update(step);//电机数据更新--------ID
			step = 0;
		}
		Motor_date_send(&Motor_send_ID);		//2--到--9号电机轮流发控制电机数据
		while(nnn<5000)
		{
			nnn++;
		}
		nnn = 0;//此处大约是一个89微秒的延时  15,000 ÷ 168,000,000 周期/秒 ≈ 89微秒
	}
}



