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

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* ================= 数学与轨迹生成工具函数 ================= */
#define PI_2 6.2831853f       // 2*PI
#define INV_PI_2 0.1591549f   // 1/(2*PI)

/**
 * @brief 生成摆线轨迹（用于抬腿相）
 * @param start_x   起始X坐标
 * @param step_len  总步长
 * @param leg_h     基础腿长（地面高度）
 * @param lift_h    最大抬腿高度
 * @param ratio     当前进度 (0.0 ~ 1.0)
 * @param out_x     输出X
 * @param out_y     输出Y
 */
static inline void Get_Cycloid_Point(float start_x, float step_len, float leg_h, float lift_h, float ratio, float *out_x, float *out_y)
{
    // X轴：标准摆线方程 x = r(t - sin(t)) 的变体 -> 映射到线性进度
    // Original: -start + step * (ratio - 1/(2pi)*sin(2pi*ratio))
    *out_x = start_x + step_len * (ratio - INV_PI_2 * sinf(PI_2 * ratio));
    
    // Y轴：余弦隆起 
    // Original: high - lift * (0.5 - 0.5*cos(2pi*ratio))
    *out_y = leg_h - lift_h * (0.5f - 0.5f * cosf(PI_2 * ratio));
}

/**
 * @brief 生成线性轨迹（用于支撑相/摩擦相）
 * @param start_x   起始X坐标
 * @param end_x     终点X坐标
 * @param leg_h     腿高（保持不变）
 * @param ratio     当前进度 (0.0 ~ 1.0)
 * @param out_x     输出X
 * @param out_y     输出Y
 */
static inline void Get_Linear_Point(float start_x, float end_x, float leg_h, float ratio, float *out_x, float *out_y)
{
    *out_x = start_x + (end_x - start_x) * ratio;
    *out_y = leg_h;
}

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
	// 保护1：防止除以0
    if(L < 0.001f) L = 0.001f;
	
	float L1 = 138.0f;//上腿长
	float L2 = 279.0f;//下腿长
	
    // 保护2：三角函数输入限幅 (-1.0 到 1.0)
    float asin_input = X/L;
    float acos_input = (L*L + L1*L1 - L2*L2)/(2*L1*L);
    
    float range_Leg = asinf(CLAMP(asin_input, -1.0f, 1.0f));
    float range_separate = acosf(CLAMP(acos_input, -1.0f, 1.0f));
	
	*range1 = (3.1415f - (range_separate - range_Leg))*6.33f;
	*range2 = (3.1415f - (range_separate + range_Leg))*6.33f;
}

//修改

// 全局参数变量
int leg_high;           // 站立高度
int leg_high_tai_tui;   // 抬腿高度
int start_point;        // 起点 
int step_long;          // 步长
extern int step_rate;
extern int step_cycle;
int start_point_turn;   // 转向起点 
int step_long_turn;     // 转向步长

/**************前进偏正调节***********/
extern int step_turn_flag;
float step_turn_k = 0.0004f;

// 状态机当前状态
static Robot_State_e current_state = STATE_INIT;

/**
  * @brief  根据遥控器开关配置步态参数
  */
static void Config_Gait_Params(void)
{
    // 快速模式 / 大平地
    if((rc_rc.s1 == 3) && (rc_rc.s2 == 1)){
        leg_high = 220; leg_high_tai_tui = 40; start_point = 100;
        step_long = start_point * 2; step_rate = 175;
    }
    // 中速模式
    else if((rc_rc.s1 == 3) && (rc_rc.s2 == 3)){
        leg_high = 220; leg_high_tai_tui = 40; start_point = 75;
        step_long = start_point * 2; step_rate = 175;
    }
    // 低速模式
    else if((rc_rc.s1 == 3) && (rc_rc.s2 == 2)){
        leg_high = 220; leg_high_tai_tui = 40; start_point = 75;
        step_long = start_point * 2; step_rate = 200;
    }
    // 障碍/自旋模式 (s1=2, s2=1)
    else if((rc_rc.s1 == 2) && (rc_rc.s2 == 1)){
        leg_high = 205; leg_high_tai_tui = 50; start_point = 125;
        step_long = start_point * 2; step_rate = 250;
        start_point_turn = 70; step_long_turn = start_point_turn * 2;
    }
    // 低栏模式 (s1=2, s2=3)
    else if((rc_rc.s1 == 2) && (rc_rc.s2 == 3)){
        leg_high = 205; leg_high_tai_tui = 50; start_point = 100;
        step_long = start_point * 2; step_rate = 250;
        start_point_turn = 70; step_long_turn = start_point_turn * 2;
    }
    // 默认
    else {
        leg_high = 205; leg_high_tai_tui = 50; start_point = 75;
        step_long = start_point * 2; step_rate = 250;
        start_point_turn = 70; step_long_turn = start_point_turn * 2;
    }
    step_cycle = step_rate * 2;
}

/**
  * @brief  输入映射：根据RC判断目标状态
  */
static Robot_State_e Check_RC_Command(void)
{
    // 0. 初始化优先
    if(Dog_Iinit < 2000) return STATE_INIT;

    // 1. 跳跃模式 (S1 = 1)
    if(rc_rc.s1 == 1) {
        if(rc_rc.s2 == 1) return STATE_JUMP_BIG;
        if(rc_rc.s2 == 3) return STATE_JUMP_SMALL;
        if(rc_rc.s2 == 2) return STATE_JUMP_LONG;
        return STATE_STAND; // 保护
    }

    // 2. 自旋/原地转向模式 (S1 = 2 且 摇杆横打)
    if(rc_rc.s1 == 2 && (rc_rc.ch0 > 110 || rc_rc.ch0 < -110)) {
        return STATE_SPIN_TURN;
    }

    // 3. 行走模式 (S1 = 3 或 2, 且 ch3 纵向推杆)
    if((rc_rc.s1 == 3 || rc_rc.s1 == 2) && (rc_rc.ch3 > 330 || rc_rc.ch3 < -330)) {
        return STATE_WALK;
    }

    // 4. 默认待机
    return STATE_STAND;
}

// ================== 查找表定义 ==================
// 用于自旋模式：定义不同腿组在右转时的X轴运动方向
// Index对应 direction_mode (0不用, 1~4有效)
// 1(Leg2,3) & 3(Leg4,5): 摩擦相X减小(-1)
// 2(Leg6,7) & 4(Leg8,9): 摩擦相X增加(+1)
static const int8_t TURN_X_DIR_SIGN[5] = {0, -1, 1, -1, 1};

/**
 * @brief 处理行走时的差速转向修正 (步幅缩放)
 */
static void Apply_Walk_Steering(float *X, int dir_mode, int rc_ch0)
{
    // 如果没有转向输入且标志位未置位，直接返回
    if (rc_ch0 == 0 && !step_turn_flag) return;
    
    step_turn_flag = 1; // 激活转向标志
    float turn_factor = step_turn_k * (float)rc_ch0; // 预计算系数
    if (turn_factor > 0.5f) turn_factor = 0.5f;      // 限幅保护
    if (turn_factor < -0.5f) turn_factor = -0.5f;

    // 逻辑简化：根据遥控器方向和腿的ID决定是增大还是减小步幅
    // 下面的逻辑是对原代码的等价压缩
    if (rc_ch0 > 0) // 右转
    {
        if (dir_mode == 3 || dir_mode == 2) *X *= (0.91f - turn_factor); // 内侧腿减速
        else if (dir_mode == 4 || dir_mode == 1) *X *= 1.09f;            // 外侧腿加速
    }
    else if (rc_ch0 < 0) // 左转
    {
        if (dir_mode == 4 || dir_mode == 1) *X *= (1.0f + turn_factor); // 注意 turn_factor 为负
        // 原代码中 dir_mode 2/3 在左转时部分无操作或逻辑不同，此处保留原逻辑意图
        // 建议实际调试时检查是否需要对称处理
    }
}

/**
 * @brief 计算行走轨迹 (统一了 MoveMode 1 和 2)
 */
static void Calc_Walk_Trajectory(float *X, float *Y, int Tm, int time_curr, int move_mode, int dir_mode)
{
    // 1. 预判参数
    int is_forward = (rc_rc.ch3 > 330);
    float t_ratio = (float)time_curr / (float)Tm;
    
    // 2. 状态判断：当前是【摆动相(抬腿)】还是【支撑相(着地)】？
    // MoveMode 1 (先支撑): 0~Tm 为支撑(Support), Tm~End 为摆动(Swing)
    // MoveMode 2 (先抬腿): 0~Tm 为摆动(Swing), Tm~End 为支撑(Support)
    int is_swing_phase = 0;
    
    if (move_mode == 1) {
        is_swing_phase = (time_curr > Tm);
        if (is_swing_phase) t_ratio = t_ratio - 1.0f; // 重置 Swing 阶段的时间比率 0~1
    } else {
        is_swing_phase = (time_curr <= Tm);
        if (!is_swing_phase) t_ratio = t_ratio - 1.0f; // 重置 Support 阶段的时间比率 0~1
    }

    // 3. 轨迹生成
    if (is_swing_phase) 
    {
        // === 摆动相 (Swing) ===
        // 摆动永远是从 后 -> 前 (前进时) 或 前 -> 后 (后退时)
        float start = is_forward ? -start_point : start_point;
        // 调用摆线函数
        Get_Cycloid_Point(start, (is_forward ? step_long : -step_long), leg_high, leg_high_tai_tui, t_ratio, X, Y);
    } 
    else 
    {
        // === 支撑相 (Support) ===
        // 支撑永远是直线，方向与摆动相反
        float start = is_forward ? start_point : -start_point;
        float end   = is_forward ? -start_point : start_point;
        Get_Linear_Point(start, end, leg_high, t_ratio, X, Y);
    }

    // 4. 叠加差速转向
    Apply_Walk_Steering(X, dir_mode, rc_rc.ch0);
}

/**
 * @brief 计算原地自旋轨迹 (表驱动优化版)
 */
static void Calc_Spin_Trajectory(float *X, float *Y, int turn_Tm, int time_turn, int dir_mode)
{
    float t_ratio = (float)time_turn / (float)turn_Tm;
    int is_right_turn = (rc_rc.ch0 > 110);
    
    // 1. 获取方向符号
    // base_sign: 定义了该腿在右转时的摩擦运动方向 (1: X增, -1: X减)
    int base_sign = TURN_X_DIR_SIGN[dir_mode]; 
    
    // 如果是左转，所有方向取反
    if (!is_right_turn) base_sign = -base_sign;

    // 2. 判断相位
    // 0~turn_Tm: 摩擦相 (脚贴地移动)
    // turn_Tm~End: 复位相 (抬腿回位)
    int is_reset_phase = (time_turn > turn_Tm);
    if (is_reset_phase) t_ratio -= 1.0f;

    // 3. 计算起点终点
    // 摩擦相：从 (sign * start) -> (-sign * start)
    // 复位相：从 (-sign * start) -> (sign * start) [摆线]
    float limit = (float)base_sign * start_point_turn; // 摩擦开始点

    if (!is_reset_phase) 
    {
        // === 摩擦相 (直线) ===
        // 特殊逻辑：原代码中复位高度有时是 leg_high * 1.1，有时是 leg_high
        // 这里统一简化，如需差异化可微调
        float h = (is_right_turn && (dir_mode==1||dir_mode==3)) ? leg_high * 1.1f : leg_high;
        if (!is_right_turn && (dir_mode==2||dir_mode==4)) h = leg_high * 1.1f;
        
        Get_Linear_Point(limit, -limit, h, t_ratio, X, Y);
    } 
    else 
    {
        // === 复位相 (摆线) ===
        Get_Cycloid_Point(-limit, 2.0f * limit, leg_high, leg_high_tai_tui, t_ratio, X, Y);
    }
}

/**
  * @brief  状态机主函数 (替代原 foot_track)
  */
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode)
{
    // 1. 配置步态参数
    Config_Gait_Params();

    // 2. 更新状态 (如果在跳跃中，暂时不切换状态，直到跳跃完成)
    // 只有当不在跳跃 或 跳跃计时器归零时，才允许切换状态
    if (current_state != STATE_JUMP_BIG && current_state != STATE_JUMP_SMALL && current_state != STATE_JUMP_LONG) {
        current_state = Check_RC_Command();
    } else {
        // 如果正在跳跃，但RC切换了，检查是否需要强制退出跳跃(原逻辑是s1!=1则清零)
        if(rc_rc.s1 != 1) {
            current_state = Check_RC_Command(); // 强制打断
            dog_jump_time = 0; dog_jump_flag = 0;
            dog_jump_time_2 = 0; dog_jump_flag_2 = 0;
            dog_jump_time_3 = 0; dog_jump_flag_3 = 0;
        }
    }

    // 3. 执行状态逻辑
    switch (current_state) 
    {
        case STATE_INIT:
            // 初始化复位逻辑
            if(Dog_Iinit < 2000) {
				float r = (float)Dog_Iinit / 2000.0f;
                Get_Linear_Point((direction_mode%2 ? 160:-160), 0, 40 + (leg_high-40)*r, r, X, Y);
                *X = (direction_mode%2 ? 160:-160) * (1.0f - r); // X需特殊处理覆盖
            } else {
                current_state = STATE_STAND; // 初始化完成，自动切待机
            }
            break;

        case STATE_WALK:
            Calc_Walk_Trajectory(X, Y, Tm, time_currently, move_mode, direction_mode);
            break;

        case STATE_SPIN_TURN:
            Calc_Spin_Trajectory(X, Y, turn_Tm, time_turn, direction_mode);
            break;

        case STATE_JUMP_BIG:
            dog_jump_flag = 1;
            if(dog_jump_time < 500) { *X = -50; *Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*(dog_jump_time/1000.0f))); }
            else if(dog_jump_time < 700) { *X = -220; *Y = 350; }
            else if(dog_jump_time < 900) { 
                float t = (dog_jump_time-700)/200.0f;
                float offset = 190.0f; if(direction_mode%2==0) offset = 260.0f; // 简单区分前后腿
                *X = -220 + offset*(t - 0.1591f*sin(6.283f*t));
                *Y = 350 - 190*(t - 0.1591f*sin(6.283f*t));
            }
            else if(dog_jump_time > 1400 && dog_jump_time < 1900) {
                float t = (dog_jump_time-1400)/500.0f;
                *X = -50 + 50*(t - 0.1591f*sin(6.283f*t));
                *Y = 160 + 40*(t - 0.1591f*sin(6.283f*t));
            } else { *X = 0; *Y = leg_high; } // 间隙期保持
            break;

        case STATE_JUMP_SMALL:
            dog_jump_flag_2 = 1;
            if(dog_jump_time_2 < 500) { *X = -50; *Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*(dog_jump_time_2/1000.0f))); }
            else if(dog_jump_time_2 < 700) { *X = -200; *Y = 250; }
            else if(dog_jump_time_2 < 900) {
                float t = (dog_jump_time_2-700)/200.0f;
                float off_x = 210.0f; if(direction_mode%2==0) off_x = 200.0f;
                float off_y = 50.0f; if(direction_mode%2==0) off_y = 90.0f;
                *X = -200 + off_x*(t - 0.1591f*sin(6.283f*t));
                *Y = (direction_mode%2!=0 ? 210:250) - off_y*(t - 0.1591f*sin(6.283f*t));
            }
            else if(dog_jump_time_2 > 1400 && dog_jump_time_2 < 1900) {
                 float t = (dog_jump_time_2-1400)/500.0f;
                *X = 10 - 10*(t - 0.1591f*sin(6.283f*t));
                *Y = 160 + 40*(t - 0.1591f*sin(6.283f*t));
            } else { *X = 0; *Y = leg_high; }
            break;

        case STATE_JUMP_LONG:
             dog_jump_flag_3 = 1;
             if(dog_jump_time_3 < 500) { *X = -50; *Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*(dog_jump_time_3/1000.0f))); }
             else if(dog_jump_time_3 < 700) { *X = -280; *Y = 280; }
             else if(dog_jump_time_3 < 900) {
                float t = (dog_jump_time_3-700)/200.0f;
                float off_y = 80.0f; if(direction_mode%2==0) off_y = 120.0f;
                *X = -280 + 430.0f*(t - 0.1591f*sin(6.283f*t));
                *Y = (direction_mode%2!=0 ? 240:280) - off_y*(t - 0.1591f*sin(6.283f*t));
             }
             else if(dog_jump_time_3 > 1400 && dog_jump_time_3 < 1900) {
                float t = (dog_jump_time_3-1400)/500.0f;
                *X = 150 - 150*(t - 0.1591f*sin(6.283f*t)); 
                *Y = 160 + 40*(t - 0.1591f*sin(6.283f*t)); 
             } else { *X = 0; *Y = leg_high; }
             break;

        case STATE_STAND:
        default:
			*X = 0; *Y = leg_high;
            step_turn_flag = 0;
            break;
    }
}
