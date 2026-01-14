#ifndef _MA_SU
#define	_MA_SU

#include "stm32f4xx.h"



int Motor_pid_count(int Motor_feedback_ID,float set_pos);
int Motor_Mode_Init(void);
int Motor_data_update(int Motor_feedback_ID);
int Motor_date_send(int*Motor_send_ID);
void PID_Init(void);
void Pos_speed_control(float feedback_pos,float set_pos,float pos_speed);
void counter_motion(float X,float Y,float *range1, float *range2);
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode);

#endif 

