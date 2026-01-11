//使用的电机为23，45，67，89
//串口1接大疆遥控
//串口3用于debug（可以不用，串口打印有时候识别不到，不如debug）
//串口6转485控制电机

#include "main.h"

/*以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值*/
MOTOR_send motor_control_clear; 
//DMA存储数据后，在中断中用于电机ID判断
MOTOR_recv motor_feedback_data;     //数据反馈
MOTOR_send motor2_control_data;   	//电机2控制
MOTOR_recv motor2_feedback_data;	//电机2反馈
MOTOR_send motor3_control_data;   	//电机3控制
MOTOR_recv motor3_feedback_data;	//电机3反馈
MOTOR_send motor4_control_data;   	//电机4控制
MOTOR_recv motor4_feedback_data;	//电机4反馈
MOTOR_send motor5_control_data;   	//电机5控制
MOTOR_recv motor5_feedback_data;	//电机5反馈
MOTOR_send motor6_control_data;   	//电机6控制
MOTOR_recv motor6_feedback_data;	//电机6反馈
MOTOR_send motor7_control_data;   	//电机7控制
MOTOR_recv motor7_feedback_data;	//电机7反馈
MOTOR_send motor8_control_data;   	//电机8控制
MOTOR_recv motor8_feedback_data;	//电机8反馈
MOTOR_send motor9_control_data;   	//电机8控制
MOTOR_recv motor9_feedback_data;	//电机8反馈
//---------------------------------------------
uint8_t Motor_Rxflag = 0;
uint8_t Motor_Rx_date = 0;
//---------------------------------------------

int tim_t = 1;////启动延时等待误差结束
int time_currently = 0;//步态当前时间
int time_turn = 0;//转向当前时间
int step_rate = 0;//控制周期时长

extern uint8_t RC[18];

extern int Motor_send_ID;	//2--到--9号电机轮流发控制电机数据
extern int Motor_feedback_ID ;//反馈ID
extern int step ;
//计时用
extern int Dog_Iinit;

//计算完得到的角度
float rang__2 = 0.0f;
float rang__3 = 0.0f;
float rang__4 = 0.0f;
float rang__5 = 0.0f;
float rang__6 = 0.0f;
float rang__7 = 0.0f;
float rang__8 = 0.0f;
float rang__9 = 0.0f;

//足端轨迹
//--1号腿足端轨迹坐标
float foot_track_x1 = 0.0f;
float foot_track_y1 = 0.0f;
//--2号腿足端轨迹坐标
float foot_track_x2 = 0.0f;
float foot_track_y2 = 0.0f;
//--3号腿足端轨迹坐标
float foot_track_x3 = 0.0f;
float foot_track_y3 = 0.0f;
//--4号腿足端轨迹坐标
float foot_track_x4 = 0.0f;
float foot_track_y4 = 0.0f;	

int nnn = 0; //main函数运行周期延时

extern int step_turn_flag;//步幅转向标志

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
			if((step == 2)||(step == 3))
			{		
				foot_track(&foot_track_x1,&foot_track_y1,step_rate,time_currently,200,time_turn,2,1);
				counter_motion(foot_track_x1,foot_track_y1, &rang__2, &rang__3);
			}
			if((step == 4)||(step == 5))
			{
				foot_track(&foot_track_x2,&foot_track_y2,step_rate,time_currently,200,time_turn,1,3);
				counter_motion(foot_track_x2,foot_track_y2, &rang__5, &rang__4);
			}
			if((step == 6)||(step == 7))
			{
				foot_track(&foot_track_x3,&foot_track_y3,step_rate,time_currently,200,time_turn,2,2);
				counter_motion(foot_track_x3,foot_track_y3, &rang__6, &rang__7);
			}
			if((step == 8)||(step == 9))
			{
				foot_track(&foot_track_x4,&foot_track_y4,step_rate,time_currently,200,time_turn,1,4);
				counter_motion(foot_track_x4,foot_track_y4, &rang__9, &rang__8);
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



