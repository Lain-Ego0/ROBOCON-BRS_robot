#ifndef _MOTOR_CONTROL
#define	_MOTOR_CONTROL

#include "stm32f4xx.h"
#include "pid.h"       
#include "GO_M8010_6.h"

// ==========================================
// 数据结构定义
// ==========================================

typedef struct {
    int id;                 
    float offset;           
    float dir_sign;         
    
    MOTOR_send control;     
    MOTOR_recv feedback;    
    
    pid_type_def speed_pid; 
    pid_type_def angle_pid; 
    
    float target_angle;     
} Motor_Unit_TypeDef;

extern Motor_Unit_TypeDef Leg_Motors[8]; 
extern int g_calibration_mode; // 全局校准模式标志

// ==========================================
// 函数接口
// ==========================================

void Motor_Mode_Init(void);
void PID_Init(void);
void Motor_pid_compute_all(void);    
void Motor_data_update_all(void);    
int Motor_date_send_sequential(int *current_index); 

// 核心算法
void counter_motion(float X, float Y, float *range1, float *range2);
void foot_track(float *X, float *Y, int Tm, int time_currently, int turn_Tm, int time_turn, int move_mode, int direction_mode);

#endif
