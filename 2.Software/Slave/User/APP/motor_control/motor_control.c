#include "motor_control.h" 
#include "pid.h"
#include "GO_M8010_6.h"
#include "bsp_debug_usart.h"
#include "bsp_key.h" 
#include "remote.h" 
#include "math.h"
#include <stdlib.h> 

/* -------------------------------------------------------------------------
   配置参数
   ------------------------------------------------------------------------- */
#define SPEED_P 0.01f
#define SPEED_I 0.0006f
#define SPEED_D 0.0015f    
#define RANG_P  35.9f
#define RANG_I  0.0f
#define RANG_D  1.0f
#define SPEED_MAX_OUT  3.50f
#define SPEED_MAX_IOUT 0.2f
#define RANG_MAX_OUT   10000.0f
#define RANG_MAX_IOUT  0.2f

#define LEG_L1 138.0f       
#define LEG_L2 279.0f       
#define LEG_MAX_LEN 400.0f  
#define LEG_MIN_LEN 50.0f   
#define REDUCTION_RATIO 6.33f
#define PI 3.1415926f

Motor_Unit_TypeDef Leg_Motors[8];
MOTOR_recv motor_feedback_data; 
extern MOTOR_send motor_control_clear; 

// 校准模式标志 (1=开启, 0=关闭)
int g_calibration_mode = 0;

// 外部引用
extern int step_rate; 
int step_cycle = 500; 
extern int step_turn_flag;

// 状态变量
float g_stand_progress = 0.0f; // 0.0(趴下) -> 1.0(站立)
int Dog_Iinit = 0; 

// ========================================================
// 跳跃变量实体定义
// ========================================================
int dog_jump_time = 0;
int dog_jump_flag = 0;
int dog_jump_time_2 = 0;
int dog_jump_flag_2 = 0;
int dog_jump_time_3 = 0;
int dog_jump_flag_3 = 0;

const fp32 Speed_PID_Params[3] = {SPEED_P, SPEED_I, SPEED_D};	
const fp32 Rang_PID_Params[3]  = {RANG_P,  RANG_I,  RANG_D};		

const float Motor_Offsets[8] = {
    8.652f, -9.970f, 4.984f, -13.648f, 4.397f, -17.293f, 11.656f, -8.065f
};
const float Motor_Directions[8] = {
    1.0f, 1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f
};

// 步态配置
typedef struct {
    float leg_high;
    float leg_lift_height;
    float start_point;
    float step_length;
    float start_point_turn;
    float step_length_turn;
    int   local_step_rate;
} GaitConfig_t;

static GaitConfig_t g_gait;

// -------------------------------------------------------------------------
// 初始化与 PID 函数
// -------------------------------------------------------------------------

void PID_Init(void) {
    for(int i = 0; i < 8; i++) {
        PID_init(&Leg_Motors[i].speed_pid, PID_POSITION, Speed_PID_Params, SPEED_MAX_OUT, SPEED_MAX_IOUT);
        PID_init(&Leg_Motors[i].angle_pid, PID_POSITION, Rang_PID_Params,  RANG_MAX_OUT,  RANG_MAX_IOUT);
    }
}

void Motor_Mode_Init(void) {
    motor_control_clear.id = 15; motor_control_clear.mode = 1; modify_data(&motor_control_clear);
    for(int i = 0; i < 8; i++) {
        Leg_Motors[i].id = i + 2;
        Leg_Motors[i].offset = Motor_Offsets[i];
        Leg_Motors[i].dir_sign = Motor_Directions[i];
        Leg_Motors[i].control.id = Leg_Motors[i].id;
        Leg_Motors[i].control.mode = 1;
        modify_data(&Leg_Motors[i].control);
    }
}

// PID计算函数：增加校准模式判断
void Motor_pid_compute_all(void) {
    for(int i = 0; i < 8; i++) {
        // 如果处于校准模式，强制输出力矩为0，方便手动掰腿
        if (g_calibration_mode == 1) {
            Leg_Motors[i].control.T = 0.0f;
            // 清除PID积分项，防止退出校准时突变
            Leg_Motors[i].speed_pid.Iout = 0;
            Leg_Motors[i].angle_pid.Iout = 0;
            continue; 
        }

        float real_pos = Leg_Motors[i].feedback.Pos + Leg_Motors[i].offset;
        float target = Leg_Motors[i].target_angle * Leg_Motors[i].dir_sign;
        float speed_target = PID_calc(&Leg_Motors[i].angle_pid, real_pos, target);
        Leg_Motors[i].control.T = PID_calc(&Leg_Motors[i].speed_pid, Leg_Motors[i].feedback.W, speed_target);
    }
}

void Motor_data_update_all(void) {
    for(int i = 0; i < 8; i++) modify_data(&Leg_Motors[i].control);
}

int Motor_date_send_sequential(int *current_index) {
    if (*current_index < 0 || *current_index > 7) *current_index = 0;
    Usart_SendStr_length(USART6, ((uint8_t *)(&Leg_Motors[*current_index].control)), Leg_Motors[*current_index].control.hex_len);
    (*current_index)++;
    if (*current_index > 7) *current_index = 0;
    return 0;
}

void counter_motion(float X, float Y, float *range1, float *range2) {
    float L_sq = X * X + Y * Y;
    float L = sqrtf(L_sq);
    if (L > LEG_MAX_LEN) L = LEG_MAX_LEN;
    if (L < LEG_MIN_LEN) L = LEG_MIN_LEN;

    float sin_arg = X / L;
    if (sin_arg > 1.0f) sin_arg = 1.0f; if (sin_arg < -1.0f) sin_arg = -1.0f;
    float range_Leg = asinf(sin_arg);

    float cos_arg = (L * L + LEG_L1 * LEG_L1 - LEG_L2 * LEG_L2) / (2.0f * LEG_L1 * L);
    if (cos_arg > 1.0f) cos_arg = 1.0f; if (cos_arg < -1.0f) cos_arg = -1.0f;
    float range_separate = acosf(cos_arg);

    *range1 = (PI - (range_separate - range_Leg)) * REDUCTION_RATIO;
    *range2 = (PI - (range_separate + range_Leg)) * REDUCTION_RATIO;
}

// -------------------------------------------------------------------------
// 状态处理逻辑
// -------------------------------------------------------------------------

static inline float Get_Cycloid(float start, float end, float t) {
    float len = start - end;
    return start - len * (t - 0.15915f * sinf(2.0f * PI * t));
}
static inline float Get_Height_Sine(float base, float lift, float t) {
    return base - lift * (0.5f - 0.5f * cosf(2.0f * PI * t));
}

// 步态参数更新
static void Update_Gait_Params(void) {
    // 默认
    g_gait.leg_high = 205.0f;
    g_gait.leg_lift_height = 50.0f;
    g_gait.start_point = 75.0f;
    g_gait.start_point_turn = 70.0f;
    g_gait.local_step_rate = 250;

    if (rc_rc.s1 == 3) { // 运动模式
        if (rc_rc.s2 == 1) { // 快速
            g_gait.local_step_rate = 175;
            g_gait.leg_high = 210; 
            g_gait.leg_lift_height = 40;
        } else if (rc_rc.s2 == 2) { // 慢速
            g_gait.local_step_rate = 300;
             g_gait.leg_high = 220;
        } else { // 中速 (S2==3)
            g_gait.local_step_rate = 220;
            g_gait.leg_high = 215;
        }
    }

    g_gait.step_length = g_gait.start_point * 2.0f;
    g_gait.step_length_turn = g_gait.start_point_turn * 2.0f;
    
    if (g_gait.local_step_rate == 0) g_gait.local_step_rate = 250;
    step_rate = g_gait.local_step_rate;
    step_cycle = step_rate * 2;
}

// 系统模式处理
static void Handle_System(float *X, float *Y, int cmd_type, int dir_mode) {
    const float SPEED_FACTOR = 0.001f; // 升降速度

    // S2=1: 启动 (0 -> 1)
    if (cmd_type == 1) {
        g_stand_progress += SPEED_FACTOR;
        if (g_stand_progress > 1.0f) g_stand_progress = 1.0f;
    }
    // S2=3: 趴下 (1 -> 0)
    else if (cmd_type == 3) {
        g_stand_progress -= SPEED_FACTOR;
        if (g_stand_progress < 0.0f) g_stand_progress = 0.0f;
    }
    
    // 更新旧的显示变量
    Dog_Iinit = (int)(g_stand_progress * 2000);

    float init_x_offset = (dir_mode == 1 || dir_mode == 3) ? 160.0f : -160.0f;
    *X = init_x_offset * (1.0f - g_stand_progress);
    *Y = 40.0f + (g_gait.leg_high - 40.0f) * g_stand_progress;
}

// 运动模式处理
static void Handle_Locomotion(float *X, float *Y, int Tm, int time_now, int move_mode, int dir_mode) {
    // ch3 控制前后
    int is_fwd = (rc_rc.ch3 > 50);
    int is_bwd = (rc_rc.ch3 < -50);
    
    if (!is_fwd && !is_bwd) { *X = 0; *Y = g_gait.leg_high; return; }

    float t = (float)time_now / (float)Tm;
    if (t > 1.0f) t = 1.0f;

    float start = is_fwd ? g_gait.start_point : -g_gait.start_point;
    float end   = is_fwd ? -g_gait.start_point : g_gait.start_point;

    if (move_mode == 1) { // 支撑相
        *X = start + (end - start) * t;
        *Y = g_gait.leg_high;
    } else { // 摆动相
        *X = Get_Cycloid(end, start, t);
        *Y = Get_Height_Sine(g_gait.leg_high, g_gait.leg_lift_height, t);
    }

    // 混合转向 (Mixed Turning) 使用 ch2
    if (abs(rc_rc.ch2) > 50) {
        float mix_factor = 0.0005f * rc_rc.ch2; 
        
        if (rc_rc.ch2 > 0) { // 向右偏
            if (dir_mode == 1 || dir_mode == 4) *X *= (1.0f + mix_factor); 
            else *X *= (1.0f - mix_factor); 
        } else { // 向左偏
            if (dir_mode == 2 || dir_mode == 3) *X *= (1.0f + abs(mix_factor)); 
            else *X *= (1.0f - abs(mix_factor)); 
        }
    }
}

// 原地转向处理
static void Handle_SpotTurn(float *X, float *Y, int turn_Tm, int time_turn, int dir_mode) {
    if (abs(rc_rc.ch0) <= 110) return; 

    float t = (float)time_turn / (float)turn_Tm;
    if (t > 1.0f) t = 1.0f;
    int is_right = (rc_rc.ch0 > 0);
    int phase = (time_turn < turn_Tm) ? 0 : 1; 

    int is_driver = 0;
    if ((dir_mode == 1 || dir_mode == 2) && is_right) is_driver = 1;
    else if ((dir_mode == 3 || dir_mode == 4) && !is_right) is_driver = 1;

    float sp = g_gait.start_point_turn;
    float sign = (dir_mode == 2 || dir_mode == 4) ? -1.0f : 1.0f;
    if (dir_mode == 3 && !is_right && phase == 1) sign = -1.0f;

    if (is_driver) {
        *X = sign * sp * (1.0f - 2.0f * t);
        if (dir_mode == 2) *X = -sp + 2.0f * sp * t;
        *Y = g_gait.leg_high * 1.05f; 
    } else {
        *X = -sign * sp + 2.0f * sign * sp * Get_Cycloid(0, 1, t);
        *Y = Get_Height_Sine(g_gait.leg_high, g_gait.leg_lift_height, t);
    }
}

// 跳跃模式处理
static void Handle_Jump(float *X, float *Y, int dir_mode) {
    int *timer, *flag;
    int type = 1;

    // S2 控制类型
    if (rc_rc.s2 == 1) { // 大跳
        type=1; timer=&dog_jump_time; flag=&dog_jump_flag; 
    } else if (rc_rc.s2 == 3) { // 小跳
        type=3; timer=&dog_jump_time_2; flag=&dog_jump_flag_2; 
    } else if (rc_rc.s2 == 2) { // 跳远
        type=2; timer=&dog_jump_time_3; flag=&dog_jump_flag_3; 
    } else return;

    *flag = 1; 
    float t = (float)(*timer);
    
    const float T_SQUAT = 500;
    const float T_JUMP = 700;
    const float T_LAND = 900;
    const float T_RESET = 1400;

    if (t < T_SQUAT) { 
        *X = -50.0f;
        *Y = 200.0f - 50.0f * (0.5f - 0.5f * cosf(2*PI * t / 1000.0f));
    } else if (t < T_JUMP) { 
        *X = (type == 2) ? -280.0f : -220.0f; 
        *Y = (type == 2) ? 280.0f : ((type == 3) ? 250.0f : 350.0f);
    } else if (t < T_LAND) { 
        float land_t = (t - T_JUMP) / 200.0f;
        float offset = (dir_mode==1||dir_mode==3) ? 190 : 260;
        if (type==2) offset = 430;
        *X = -220.0f + offset * Get_Cycloid(0, -1, land_t); 
        *Y = 350.0f; 
    } else if (t > T_RESET && t < 1900) { 
        float reset_t = (t - T_RESET) / 500.0f;
        *X = 0; 
        *Y = 200.0f; 
    }
}

// ==========================================
// 主入口函数：逻辑总控
// ==========================================
void foot_track(float *X, float *Y, int Tm, int time_currently, int turn_Tm, int time_turn, int move_mode, int direction_mode)
{
    Update_Gait_Params();
    g_calibration_mode = 0; // 默认关闭

    // 状态机仲裁
    enum { S_SYS_START, S_SYS_DOWN, S_SYS_CALIB, S_JUMP, S_SPOT_TURN, S_MOVE, S_IDLE };
    int state = S_IDLE;

    if (rc_rc.s1 == 2) { // [S1=2] 系统模式
        if (rc_rc.s2 == 1) state = S_SYS_START;       // 启动
        else if (rc_rc.s2 == 3) state = S_SYS_DOWN;   // 趴下
        else if (rc_rc.s2 == 2) state = S_SYS_CALIB;  // 校准
    } 
    else if (rc_rc.s1 == 1) { // [S1=1] 跳跃模式
        state = S_JUMP; 
    }
    else if (rc_rc.s1 == 3) { // [S1=3] 运动模式
        if (abs(rc_rc.ch0) > 110) state = S_SPOT_TURN; // 原地转向
        else state = S_MOVE; // 前进/后退/混合转向
    }

    // 执行逻辑
    switch (state) {
        case S_SYS_START:
            Handle_System(X, Y, 1, direction_mode); 
            break;
        case S_SYS_DOWN:
            Handle_System(X, Y, 3, direction_mode); 
            break;
        case S_SYS_CALIB:
            g_calibration_mode = 1; 
            *X = 0; *Y = 150; 
            break;
        case S_JUMP:
            Handle_Jump(X, Y, direction_mode);
            break;
        case S_SPOT_TURN:
            Handle_SpotTurn(X, Y, turn_Tm, time_turn, direction_mode);
            break;
        case S_MOVE:
            Handle_Locomotion(X, Y, Tm, time_currently, move_mode, direction_mode);
            break;
        case S_IDLE:
        default:
            *X = 0.0f;
            *Y = g_gait.leg_high;
            dog_jump_time = 0; dog_jump_flag = 0;
            dog_jump_time_2 = 0; dog_jump_flag_2 = 0;
            dog_jump_time_3 = 0; dog_jump_flag_3 = 0;
            break;
    }
}
