#include "motor_control.h" 
#include "pid.h"
#include "GO_M8010_6.h"
#include "bsp_debug_usart.h"
#include "bsp_key.h" 
#include "remote.h"
#include "math.h"

/*PID参数final 2025-7-13*/
#define speed_p 0.01f
#define speed_i 0.0006f
#define speed_d 0.0015f    
#define rang_p 5.9f
#define rang_i 0.0f
#define rang_d 1.0f
#define speed_max_out 3.50
#define speed_max_iout 0.2
#define rang_max_out 10000
#define rang_max_iout 0.2

//以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
extern	MOTOR_send motor_control_clear; 	// 电机停止
/*----------------------------------------------------*/
extern	MOTOR_send motor2_control_data;   	// 电机2控制
extern	MOTOR_recv motor2_feedback_data;	// 电机2反馈
extern	MOTOR_send motor3_control_data;   	// 电机3控制
extern	MOTOR_recv motor3_feedback_data;	// 电机3反馈
extern	MOTOR_send motor4_control_data;   	// 电机4控制
extern	MOTOR_recv motor4_feedback_data;	// 电机4反馈
extern	MOTOR_send motor5_control_data;   	// 电机5控制
extern	MOTOR_recv motor5_feedback_data;	// 电机5反馈
extern	MOTOR_send motor6_control_data;   	// 电机6控制
extern	MOTOR_recv motor6_feedback_data;	// 电机6反馈
extern	MOTOR_send motor7_control_data;   	// 电机7控制
extern	MOTOR_recv motor7_feedback_data;	// 电机7反馈
extern	MOTOR_send motor8_control_data;   	// 电机8控制
extern	MOTOR_recv motor8_feedback_data;	// 电机8反馈
extern	MOTOR_send motor9_control_data;   	// 电机9控制
extern	MOTOR_recv motor9_feedback_data;	// 电机9反馈

float Motor2_speed_PID_OUT = 0;float Motor2_rang_PID_OUT  = 0;
float Motor3_speed_PID_OUT = 0;float Motor3_rang_PID_OUT  = 0;
float Motor4_speed_PID_OUT = 0;float Motor4_rang_PID_OUT  = 0;
float Motor5_speed_PID_OUT = 0;float Motor5_rang_PID_OUT  = 0;
float Motor6_speed_PID_OUT = 0;float Motor6_rang_PID_OUT  = 0;
float Motor7_speed_PID_OUT = 0;float Motor7_rang_PID_OUT  = 0;
float Motor8_speed_PID_OUT = 0;float Motor8_rang_PID_OUT  = 0;
float Motor9_speed_PID_OUT = 0;float Motor9_rang_PID_OUT  = 0;

//初始化PID参数
const fp32 Motor2_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor2_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor3_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor3_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor4_speed_PID_data[3] = {speed_p,speed_i,speed_d};	 
const fp32 Motor4_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor5_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor5_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor6_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor6_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor7_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor7_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		
const fp32 Motor8_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor8_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		 
const fp32 Motor9_speed_PID_data[3] = {speed_p,speed_i,speed_d};	
const fp32 Motor9_rang_PID_data[3]  = {rang_p,rang_i,rang_d};		

extern float rang__2;
extern float rang__3;
extern float rang__4;
extern float rang__5;
extern float rang__6;
extern float rang__7;
extern float rang__8;
extern float rang__9;

/*足端轨迹*/
extern float foot_track_x1 ;
extern float foot_track_y1 ;
extern float foot_track_x2 ;
extern float foot_track_y2 ;
extern float foot_track_x3 ;
extern float foot_track_y3 ;
extern float foot_track_x4 ;
extern float foot_track_y4 ;

/*****************计时与计时标志（f4xx_it.c中断1ms计时）*****************/
extern int Dog_Iinit ;//启动初始化
extern int Dog_flag ;//启动标志位
extern int dog_jump_time;	//跳--计时1	
extern int dog_jump_time_2 ;//跳--计时2	
extern int dog_jump_time_3 ;//跳--计时3	
extern int dog_jump_flag;	//跳开始计时标志1
extern int dog_jump_flag_2 ;//跳开始计时标志2	
extern int dog_jump_flag_3 ;//跳开始计时标志3	
extern int backflip_flag ;	//空翻开始计时标志	
extern int backflip_time ;	//空翻计时
extern int time_currently;//步态当前时间
extern int time_turn ;//转向当前时间

int step = 1;//中断中赋值

int Motor_send_ID = 2;//2--到--9号电机轮流发控制电机数据
/*********初始化PID*********/
void PID_Init(void)
{
	PID_init(&Motor2_speed_PID, PID_POSITION,Motor2_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor2_rang_PID, PID_POSITION,Motor2_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor3_speed_PID, PID_POSITION,Motor3_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor3_rang_PID, PID_POSITION,Motor3_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor4_speed_PID, PID_POSITION,Motor4_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor4_rang_PID, PID_POSITION,Motor4_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor5_speed_PID, PID_POSITION,Motor5_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor5_rang_PID, PID_POSITION,Motor5_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor6_speed_PID, PID_POSITION,Motor6_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor6_rang_PID, PID_POSITION,Motor6_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor7_speed_PID, PID_POSITION,Motor7_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor7_rang_PID, PID_POSITION,Motor7_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor8_speed_PID, PID_POSITION,Motor8_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor8_rang_PID, PID_POSITION,Motor8_rang_PID_data,rang_max_out,rang_max_iout);
	PID_init(&Motor9_speed_PID, PID_POSITION,Motor9_speed_PID_data, speed_max_out, speed_max_iout);
	PID_init(&Motor9_rang_PID, PID_POSITION,Motor9_rang_PID_data,rang_max_out,rang_max_iout);
}
/***********电机模式初始化**************/
int Motor_Mode_Init(void)
{
	motor_control_clear.id=15; 			
	motor_control_clear.mode=1;
	motor_control_clear.T=0;//控制时，只对转矩更新！！！！！！！！！！！！！
	motor_control_clear.W=0.0;
	motor_control_clear.Pos=0.0;
	motor_control_clear.K_P=0.0;
	motor_control_clear.K_W=0.0;
	modify_data(&motor_control_clear);//把停机数据写入发送结构体数据赋值
	//------------------------------------
	motor2_control_data.id=2;
	motor2_control_data.mode=1;
	motor2_control_data.T=0;
	motor2_control_data.W=0.0;
	motor2_control_data.Pos=0.0;
	motor2_control_data.K_P=0.0;
	motor2_control_data.K_W=0.0;
	modify_data(&motor2_control_data);
	//------------------------------------
	motor3_control_data.id=3;
	motor3_control_data.mode=1;
	motor3_control_data.T=0;
	motor3_control_data.W=0.0;
	motor3_control_data.Pos=0.0;
	motor3_control_data.K_P=0.0;
	motor3_control_data.K_W=0.0;
	modify_data(&motor3_control_data);
	//------------------------------------
	motor4_control_data.id=4;
	motor4_control_data.mode=1;
	motor4_control_data.T=0;
	motor4_control_data.W=0.0;
	motor4_control_data.Pos=0.0;
	motor4_control_data.K_P=0.0;
	motor4_control_data.K_W=0.0;
	modify_data(&motor4_control_data);
	//------------------------------------
	motor5_control_data.id=5;
	motor5_control_data.mode=1;
	motor5_control_data.T=0;
	motor5_control_data.W=0.0;
	motor5_control_data.Pos=0.0;
	motor5_control_data.K_P=0.0;
	motor5_control_data.K_W=0.0;
	modify_data(&motor5_control_data);
	//------------------------------------
	motor6_control_data.id=6;
	motor6_control_data.mode=1;
	motor6_control_data.T=0;
	motor6_control_data.W=0.0;
	motor6_control_data.Pos=0.0;
	motor6_control_data.K_P=0.0;
	motor6_control_data.K_W=0.0;
	modify_data(&motor6_control_data);
	//------------------------------------
	motor7_control_data.id=7;
	motor7_control_data.mode=1;
	motor7_control_data.T=0;
	motor7_control_data.W=0.0;
	motor7_control_data.Pos=0.0;
	motor7_control_data.K_P=0.0;
	motor7_control_data.K_W=0.0;
	modify_data(&motor7_control_data);
	//------------------------------------
	motor8_control_data.id=8;
	motor8_control_data.mode=1;
	motor8_control_data.T=0;
	motor8_control_data.W=0.0;
	motor8_control_data.Pos=0.0;
	motor8_control_data.K_P=0.0;
	motor8_control_data.K_W=0.0;
	modify_data(&motor8_control_data);
	//------------------------------------
	motor9_control_data.id=9;
	motor9_control_data.mode=1;
	motor9_control_data.T=0;
	motor9_control_data.W=0.0;
	motor9_control_data.Pos=0.0;
	motor9_control_data.K_P=0.0;
	motor9_control_data.K_W=0.0;
	modify_data(&motor9_control_data);
	//------------------------------------
	return 0;
}

//设置反馈角度误差。本处理理论上可自动化
//角度为正侧减去角度值，角度为负侧加上角度值
#define ID2_error 8.652f
#define ID3_error 9.970f
#define ID4_error 4.984f
#define ID5_error 13.648f
#define ID6_error 4.397f
#define ID7_error 17.293f
#define ID8_error 11.656f
#define ID9_error 8.065f

/*********************电机PID计算********************/
int Motor_pid_count(int Motor_feedback_ID,float set_pos)
{
	switch(Motor_feedback_ID)
	{
		case 2 :  
			//电机速度pid-1-计算-----------------------------反馈--------------------------------目标值---			
			Motor2_rang_PID_OUT  = PID_calc(&Motor2_rang_PID, motor2_feedback_data.Pos + ID2_error,rang__2);		
			Motor2_speed_PID_OUT = PID_calc(&Motor2_speed_PID, motor2_feedback_data.W ,Motor2_rang_PID_OUT);
			break;
		case 3 :  
			Motor3_rang_PID_OUT  = PID_calc(&Motor3_rang_PID,motor3_feedback_data.Pos - ID3_error,rang__3);
			Motor3_speed_PID_OUT = PID_calc(&Motor3_speed_PID, motor3_feedback_data.W ,Motor3_rang_PID_OUT);
			break;
		case 4 :  
			Motor4_rang_PID_OUT  = PID_calc(&Motor4_rang_PID,motor4_feedback_data.Pos + ID4_error,-rang__4);
			Motor4_speed_PID_OUT = PID_calc(&Motor4_speed_PID, motor4_feedback_data.W ,Motor4_rang_PID_OUT);
			break;		
		case 5 :  
			Motor5_rang_PID_OUT  = PID_calc(&Motor5_rang_PID,motor5_feedback_data.Pos - ID5_error,-rang__5);
			Motor5_speed_PID_OUT = PID_calc(&Motor5_speed_PID, motor5_feedback_data.W ,Motor5_rang_PID_OUT);
			break;
		case 6 :  
			Motor6_rang_PID_OUT  = PID_calc(&Motor6_rang_PID,motor6_feedback_data.Pos + ID6_error,-rang__6);
			Motor6_speed_PID_OUT = PID_calc(&Motor6_speed_PID, motor6_feedback_data.W ,Motor6_rang_PID_OUT);
			break;
		case 7 :  
			Motor7_rang_PID_OUT  = PID_calc(&Motor7_rang_PID,motor7_feedback_data.Pos - ID7_error,-rang__7);
			Motor7_speed_PID_OUT = PID_calc(&Motor7_speed_PID, motor7_feedback_data.W ,Motor7_rang_PID_OUT);
			break;
		case 8 :  
			Motor8_rang_PID_OUT  = PID_calc(&Motor8_rang_PID,motor8_feedback_data.Pos + ID8_error,rang__8);
			Motor8_speed_PID_OUT = PID_calc(&Motor8_speed_PID, motor8_feedback_data.W ,Motor8_rang_PID_OUT);
			break;
		case 9 :  
			Motor9_rang_PID_OUT  = PID_calc(&Motor9_rang_PID,motor9_feedback_data.Pos - ID9_error,rang__9);
			Motor9_speed_PID_OUT = PID_calc(&Motor9_speed_PID, motor9_feedback_data.W ,Motor9_rang_PID_OUT);
			break;
	}
	return 0;
}

/*********************更新电机数据********************/
int Motor_data_update(int Motor_feedback_ID)
{
	switch(Motor_feedback_ID) //给电机控制指令结构体赋值
	{
		case 2 :  
			motor2_control_data.T = Motor2_speed_PID_OUT;
			modify_data(&motor2_control_data);
			break;
		case 3 :  
			motor3_control_data.T = Motor3_speed_PID_OUT;
			modify_data(&motor3_control_data);
			break;
		case 4 :  
			motor4_control_data.T = Motor4_speed_PID_OUT;
			modify_data(&motor4_control_data);
			break;	
		case 5 : 
			motor5_control_data.T = Motor5_speed_PID_OUT;
			modify_data(&motor5_control_data);
			break;  
		case 6 :  
			motor6_control_data.T = Motor6_speed_PID_OUT;
			modify_data(&motor6_control_data);
			break;
		case 7 :  
			motor7_control_data.T = Motor7_speed_PID_OUT;
			modify_data(&motor7_control_data);
			break;
		case 8 :  
			motor8_control_data.T= Motor8_speed_PID_OUT;
			modify_data(&motor8_control_data);
			break;
		case 9 :  
			motor9_control_data.T= Motor9_speed_PID_OUT;
			modify_data(&motor9_control_data);
			break;
		
//		case 2 :  
//			motor2_control_data.T = 0;
//			modify_data(&motor2_control_data);
//			break;
//		case 3 :  
//			motor3_control_data.T = 0;
//			modify_data(&motor3_control_data);
//			break;
//		case 4 :  
//			motor4_control_data.T = 0;
//			modify_data(&motor4_control_data);
//			break;	
//		case 5 : 
//			motor5_control_data.T = 0;
//			modify_data(&motor5_control_data);
//			break;  
//		case 6 :  
//			motor6_control_data.T = 0;
//			modify_data(&motor6_control_data);
//			break;
//		case 7 :  
//			motor7_control_data.T = 0;
//			modify_data(&motor7_control_data);
//			break;
//		case 8 :  
//			motor8_control_data.T= 0;
//			modify_data(&motor8_control_data);
//			break;
//		case 9 :  
//			motor9_control_data.T= 0;
//			modify_data(&motor9_control_data);
//			break;
		
	}
	return 0;
}

/*********************发送电机数据********************/
int Motor_date_send(int*Motor_send_ID)
{
	switch(*Motor_send_ID)
	{
		case 2 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor2_control_data)),motor2_control_data.hex_len );
			break;
		case 3 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor3_control_data)),motor3_control_data.hex_len );
			break;
		case 4 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor4_control_data)),motor4_control_data.hex_len );
			break;
		case 5 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor5_control_data)),motor5_control_data.hex_len );
			break;
		case 6 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor6_control_data)),motor6_control_data.hex_len );
			break;
		case 7 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor7_control_data)),motor7_control_data.hex_len );
			break;
		case 8 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor8_control_data)),motor8_control_data.hex_len );
			break; 
		case 9 :
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor9_control_data)),motor9_control_data.hex_len );
			break; 
	}
	if(*Motor_send_ID == 9) (*Motor_send_ID) = 1;
	*Motor_send_ID = (*Motor_send_ID) +1;
	return 0;
}

//腿关节运动逆解
void counter_motion(float X,float Y,float *range1, float *range2)
{
	float L = sqrt(X*X + Y*Y);//虚腿长
	float L1 = 138.0f;//上腿长
	float L2 = 279.0f;//下腿长
	float range_Leg = 0.0f;//腿角
	float range_separate = 0.0f;//分离角
	range_Leg = asin(X/L);
    range_separate = acos((L*L + L1*L1 - L2*L2)/(2*L1*L));
	*range1 = (3.1415f - (range_separate - range_Leg))*6.33f;
	*range2 = (3.1415f - (range_separate + range_Leg))*6.33f;
}

/**************************************/
int leg_high;// 站立高度
int leg_high_tai_tui;// 抬腿高度
int start_point;// 起点 
int step_long;// 步长
extern int step_rate;
extern int step_cycle;
int start_point_turn;// 起点 
int step_long_turn;// 步长
/**************************************/

/**************前进偏正调节***********/
extern int step_turn_flag;
float step_turn_k = 0.0004f;

//足端轨迹------------------------抬脚时长-------当前时间------转向半周期--转向当前时间-----行走与支撑-------模式------
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode)
{
	
	//-模式选择-
	if((rc_rc.s1 == 3)&&(rc_rc.s2 == 1)){	    //速度快，需要调试缓启动  用于快速过大平地
		leg_high = 220; //虚拟腿长
		leg_high_tai_tui = 40;//抬腿高度
		start_point = 100;//起点
		step_long = start_point*2;//步长
		step_rate = 175;//控制周期时长
		step_cycle = step_rate*2;			
	}else if((rc_rc.s1 == 3)&&(rc_rc.s2 == 3)){ //速度中，右前推为低腿高
		leg_high = 220; 
		leg_high_tai_tui = 40;//抬腿高度
		start_point = 75;//起点
		step_long = start_point*2;//步长
		step_rate = 175;
		step_cycle = step_rate*2;	
	}else if((rc_rc.s1 == 3)&&(rc_rc.s2 == 2)){ //速度低，右前推为低腿高
		leg_high = 220;
		leg_high_tai_tui = 40;//抬腿高度
		start_point = 75;//起点
		step_long = start_point*2;//步长 
		step_rate = 200;
		step_cycle = step_rate*2;	
	}else if((rc_rc.s1 == 2)&&(rc_rc.s2 == 1)){	//直接跑障碍         这部分拥有原地转弯功能
		leg_high = 205; //虚拟腿长
		leg_high_tai_tui = 50;//抬腿高度
		start_point = 125;//起点
		step_long = start_point*2;//步长
		step_rate = 250;
		step_cycle = step_rate*2;		
		start_point_turn = 70;// 起点 
		step_long_turn = start_point_turn*2;// 步长		
	}else if((rc_rc.s1 == 2)&&(rc_rc.s2 == 3)){	//低栏使用，右前推为低腿高
		leg_high = 205; //虚拟腿长
		leg_high_tai_tui = 50;//抬腿高度
		start_point = 100;//起点
		step_long = start_point*2;//步长
		step_rate = 250;
		step_cycle = step_rate*2;	
		start_point_turn = 70;// 起点 
		step_long_turn = start_point_turn*2;// 步长			
	}else{                                       
		leg_high = 205;
		leg_high_tai_tui = 50;//抬腿高度
		start_point = 75;//起点
		step_long = start_point*2;//步长
		step_rate = 250;
		step_cycle = step_rate*2;
		start_point_turn = 70;// 起点 
		step_long_turn = start_point_turn*2;// 步长	
	}
	
    //不进模式时，复位用（进入if触发为0，退出if触发为1————添加低栏设置）
	int flag = 1;
	//狗初始化站立起来2s  放置位置X=160 Y=40——X=0，Y=200
	if(Dog_Iinit <2000)//初始化足端位置
	{
		if(direction_mode == 1)//2-3
		{
			*X = 160 - 160*((float)Dog_Iinit/2000);
			*Y = 40 + (leg_high-40)*((float)Dog_Iinit/2000);
		}
		if(direction_mode == 2)//6-7
		{
			*X = -160 + 160*((float)Dog_Iinit/2000);
			*Y = 40 + (leg_high-40)*((float)Dog_Iinit/2000);
		}
		if(direction_mode == 3)//4-5
		{
			*X = 160 - 160*((float)Dog_Iinit/2000);
			*Y = 40 + (leg_high-40)*((float)Dog_Iinit/2000);
		}
		if(direction_mode == 4)//8-9
		{
			*X = -160 + 160*((float)Dog_Iinit/2000);
			*Y = 40 + (leg_high-40)*((float)Dog_Iinit/2000);
		}
	}else{
	/************************************************前进后退******************************************************/
	//-1-//行走先后脚------先支撑
	if(move_mode == 1)
	{
		//向前走
		if(((rc_rc.s1 == 3)||(rc_rc.s1 == 2))&&(rc_rc.ch3>330))
		{
			flag = 0;
			//抬脚时间-----抬脚
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//足端直线向后轨迹
				*X = start_point - step_long*((float)time_currently/Tm) ;
				*Y = leg_high;
			}
			else//过了抬脚时间后，足端支撑直线向后
			{
				//无四脚着地时间
				//足端摆线向前轨迹
				//起始点-----步长------------当前时间--抬脚时长
				*X =   -start_point  +   step_long*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//足端高             抬脚高度
				*Y =   leg_high  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));
			}
            //---步幅转向
			if((rc_rc.ch0 > 0)||(step_turn_flag))//转向切换
			{
				step_turn_flag = 1;
				//-------腿2号-4-5-----------步幅转向标志
				if(((direction_mode ==3))&&step_turn_flag)
				{
					//步幅转向系数step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
				//-------腿4号-8-9-----------步幅转向标志
				if(((direction_mode ==4))&&step_turn_flag)
				{
					*X = (*X)*1.09f ;
				}
			}
			if((rc_rc.ch0 < 0)||(step_turn_flag))//转向切换
			{
				step_turn_flag = 1;
				//-------腿-4-5-----------步幅转向标志
				if(((direction_mode ==3))&&step_turn_flag)
				{
					*X = *X ;
				}
				//-------腿-8-9-----------步幅转向标志
				if(((direction_mode ==4))&&step_turn_flag)
				{
					//步幅转向系数step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*( - rc_rc.ch0));
				}
			}
		}
		//向后走
		if(((rc_rc.s1 == 3)||(rc_rc.s1 == 2))&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//抬脚时间抬脚
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//足端直线向后轨迹
				*X = -start_point + step_long*((float)time_currently/Tm);
				*Y = leg_high;
			}
			else//过了抬脚时间后，足端支撑直线向后
			{
				//无四脚着地时间
				//足端摆线向前轨迹
				//起始点-----步长------------当前时间--抬脚时长
				*X =   start_point - step_long*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//足端高     抬脚高度
				*Y =   leg_high  -  leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));
			}
		}
	}
    //-1-//行走先后脚--先抬腿
	if(move_mode == 2)
	{
		//向前走
		if(((rc_rc.s1 == 3)||(rc_rc.s1 == 2))&&(rc_rc.ch3>330))
		{
			flag = 0;
			//抬脚时间-----抬脚
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//足端摆线向前轨迹
				//起始点-----步长------------当前时间--抬脚时长
				*X =   -start_point + step_long*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//足端高     抬脚高度
				*Y =   leg_high  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//过了抬脚时间后，足端支撑直线向后
			{
				//无四脚着地时间
				//足端直线向后轨迹
				*X = start_point - step_long*((float)time_currently/Tm -1.0f);
				*Y = leg_high;
			}
            //---步幅转向
			//右转--
			if((rc_rc.ch0 > 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------腿1号-2-3----------步幅转向标志
				if(((direction_mode ==1))&&step_turn_flag)
				{
					*X = (*X)*1.09f  ;
				}
				//-------腿2号-6-7----------步幅转向标志
				if(((direction_mode ==2))&&step_turn_flag)
				{
					//步幅转向系数step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
			}
			//左转--
			if((rc_rc.ch0 < 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------腿-2-3-----------步幅转向标志
				if(((direction_mode ==1))&&step_turn_flag)
				{
					//步幅转向系数step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*(-rc_rc.ch0));
				}
				//-------腿-4-5-----------步幅转向标志
				if(((direction_mode ==2))&&step_turn_flag)
				{
					* X = *X;
				}
			}
		}
		//向后走
		if(((rc_rc.s1 == 3)||(rc_rc.s1 == 2))&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//抬脚时间抬脚
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//足端摆线向前轨迹
				//起始点-----步长------------当前时间--抬脚时长
				*X =   start_point - step_long*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//足端高     抬脚高度
				*Y =   leg_high  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//过了抬脚时间后，足端支撑直线向后
			{
				//无四脚着地时间
				//足端直线向后轨迹
				*X = -start_point + step_long*((float)time_currently/Tm - 1.0f);
				*Y = leg_high;
			}
		}
	}
	/**************************************************转向********************************************************/
    //-2-//转向
	if(rc_rc.s1 == 2)
	{
		//先向前----- -X 
		if(direction_mode == 1) // 2-3
		{
			//向右转----X-正值
			if(rc_rc.ch0 > 110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端直线向后轨迹
					*X = start_point_turn-step_long_turn*((float)time_turn/turn_Tm);
					*Y = leg_high*1.1 ;
				}
				else//抬脚复位阶段
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
			}
             //向左转----X-负值
			if(rc_rc.ch0 < -110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = start_point_turn-step_long_turn*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//抬脚复位阶段
				{
					//足端直线向后轨迹
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm - 1);
					*Y = leg_high*1.1;
				}
			}
		}
         //先向前----- +X   
		if(direction_mode == 2) // 6-7
		{
            //向右转----X-正值
			if(rc_rc.ch0 > 110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端直线向后轨迹
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm);
					*Y = leg_high;
				}
				else//抬脚复位阶段
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = start_point_turn - step_long_turn*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//足端高     
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
			}
            //向左转----X-负值
			if(rc_rc.ch0 < -110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//抬脚复位阶段
				{
					//足端直线向后轨迹
					*X = start_point_turn - step_long_turn*((float)time_turn/turn_Tm - 1);
					*Y = leg_high*1.1;
				}
			}
		}
		//先抬----- -X     
		if(direction_mode == 3) // 4-5
		{
            //向右转----X-负值
			if(rc_rc.ch0 > 110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = start_point_turn - step_long_turn*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//抬脚复位阶段
				{
					//足端直线向后轨迹
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm -1 );
					*Y = leg_high*1.1;
				}
			}
            //向左转----X-负值
			if(rc_rc.ch0 < -110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_currently > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端直线向后轨迹
					*X = start_point_turn - step_long_turn*((float)time_turn/turn_Tm  );
					*Y = leg_high*1.1;
				}
				else//抬脚复位阶段
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
			}
		}
		//先抬----- +X   
		if(direction_mode == 4) // 8-9
		{
            //向右转----X-负值
			if(rc_rc.ch0 > 110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = -start_point_turn + step_long_turn*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//抬脚复位阶段
				{
					//足端直线向后轨迹
					*X = start_point_turn - step_long_turn*((float)time_turn/turn_Tm -1);
					*Y = leg_high*1.1;
				}
			}
            //向左转----X-负值
			if(rc_rc.ch0 < -110)
			{
				flag = 0;
				//摩擦力矩阶段
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<Tm)||(Tm == time_turn)))
				{
					//足端直线向后轨迹
					*X = - start_point_turn + step_long_turn*((float)time_turn/Tm  );
					*Y = leg_high*1.1;
				}
				else//抬脚复位阶段
				{
					//足端摆线向前轨迹
					//起始点-----步长------------当前时间--抬脚时长
					*X = start_point_turn - step_long_turn*((float)time_turn/Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/Tm - 1)));
					//足端高     抬脚高度
					*Y = leg_high - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/Tm - 1)));
				}
			}
		}
	}
    /**************************************************跳跃模式****************************************************/
	if(rc_rc.s1 == 1)
	{
		flag = 0; //退出复位模式（X=0 Y=腿高）
		/****************大跳跃****************/
		if(rc_rc.s2 == 1)
		{
			dog_jump_flag = 1;//跳开始计时标志
			if(500>dog_jump_time)
			{
				//下蹲
				*X = -50;
				//下蹲高度
				*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time/1000)));
			}
			if((500<dog_jump_time)&&(700>dog_jump_time))
			{
				//起跳
				*X = -220;
				*Y = 350;
			}
			//落地姿势
			if((700<dog_jump_time)&&(900>dog_jump_time))
			{
				//前腿
				if((direction_mode == 1)||(direction_mode == 3))
				{
					*X = -220 + 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
				}
				//后腿
				if((direction_mode == 2)||(direction_mode == 4))
				{
					*X = -220 + 260*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
				}
			}
			//跳完后，复位
			if((1400<dog_jump_time)&&(1900>dog_jump_time))
			{
				*X = -50 + 50*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
				*Y = 160 + 40*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
			}

		}else{ //--进入其他模式，跳时间清零
			dog_jump_time = 0;
			dog_jump_flag = 0;
		}
		
		/***************小跳跃****************/
		if(rc_rc.s2 == 3)
		{
			dog_jump_flag_2 = 1;//跳开始计时标志
			if(500>dog_jump_time_2)
			{
				//下蹲
				*X = -50;
				//下蹲高度
				*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_2/1000)));
			}
			if((500<dog_jump_time_2)&&(700>dog_jump_time_2))
			{
				//起跳
				*X = -200;
				*Y = 250;
			}
			//落地姿势--
			if((700<dog_jump_time_2)&&(900>dog_jump_time_2))
			{
				//前腿
				if((direction_mode == 1)||(direction_mode == 3))
				{
					*X = -200 + 210*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					//空中下蹲高度
					*Y = 210 - 50*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
				}
				//后腿
				if((direction_mode == 2)||(direction_mode == 4))
				{
					*X = -200 + 200*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					//空中下蹲高度
					*Y = 250 - 90*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
				}
			}
			//跳完后，复位
			if((1400<dog_jump_time_2)&&(1900>dog_jump_time_2))
			{
				*X = 10 - 10*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
				*Y = 160 + 40*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
			}

		}else{ //--进入其他模式，跳时间清零
			dog_jump_time_2 = 0;
			dog_jump_flag_2 = 0;
		}
        /*****************跳远***************/
		if(rc_rc.s2 == 2)
		{
			dog_jump_flag_3 = 1;//跳开始计时标志
			if(500>dog_jump_time_3)
			{
				//下蹲
				*X = -50;
				//下蹲高度
				*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_3/1000)));
			}
			if((500<dog_jump_time_3)&&(700>dog_jump_time_3))
			{
				//起跳
				*X = -280; // 比大跳越更靠后，增加水平距离
				*Y = 280;  // 比大跳越低，保持重心
			}
			//落地姿势--
			if((700<dog_jump_time_3)&&(900>dog_jump_time_3))
			{
				//前腿
				if((direction_mode == 1)||(direction_mode == 3))
				{
					*X = -280 + 430*((float)(dog_jump_time_3-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-700)/200)));
					//空中下蹲高度
					*Y = 240 - 80*((float)(dog_jump_time_3-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-700)/200))); //最低150
				}
				//后腿
				if((direction_mode == 2)||(direction_mode == 4))
				{
					*X = -280 + 430*((float)(dog_jump_time_3-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-700)/200)));
					//空中下蹲高度
					*Y = 280 - 120*((float)(dog_jump_time_3-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-700)/200)));//最低150
				}
			}
			//跳完后，复位
			if((1400<dog_jump_time_3)&&(1900>dog_jump_time_3))
			{
				*X = 150 - 150*((float)(dog_jump_time_3-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-1400)/500))); //本质还是回到原始坐标，但落点改为50防止掉落
				*Y = 160 + 40*((float)(dog_jump_time_3-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_3-1400)/500))); 
			}
		}else{ //--进入其他模式，跳时间清零
			dog_jump_time_3 = 0;
			dog_jump_flag_3 = 0;
		}
	}else{ //跳跃总清零
		dog_jump_time = 0;
		dog_jump_flag = 0;
		dog_jump_time_2 = 0;
		dog_jump_flag_2 = 0;	
		dog_jump_time_3 = 0;
		dog_jump_flag_3 = 0;		
	}
	    //不进模式时，复位1
		if(flag == 1)
		{
			*Y = leg_high;
			*X = 0;
		} 
	}
}


