#ifndef _MOTOR_CONTROL
#define	_MOTOR_CONTROL

#include "stm32f4xx.h"


// --- 状态机定义 ---
typedef enum {
    STATE_INIT,         // 初始化/复位
    STATE_STAND,        // 待机/站立
    STATE_WALK,         // 行走 (包含行进中的转向)
    STATE_SPIN_TURN,    // 原地自旋/大角度转向模式
    STATE_JUMP_BIG,     // 大跳
    STATE_JUMP_SMALL,   // 小跳
    STATE_JUMP_LONG     // 跳远
} Robot_State_e;

int Motor_pid_count(int Motor_feedback_ID,float set_pos);
int Motor_Mode_Init(void);
int Motor_data_update(int Motor_feedback_ID);
int Motor_date_send(int*Motor_send_ID);
void PID_Init(void);
void Pos_speed_control(float feedback_pos,float set_pos,float pos_speed);
void counter_motion(float X,float Y,float *range1, float *range2);
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode);

#endif 

