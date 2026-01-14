#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "ris_protocol.h"

#pragma pack(1)

// 读取的当前电机 的控制数据
// 保留
typedef union
{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

typedef struct
{
    // 定义数据包头
    unsigned char start[2]; // 包头
    unsigned char motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    unsigned char reserved;
} COMHead;

typedef struct
{ 						// 以4个字节一组排列 ，不然编译器会凑整
    // 定义数据
    uint8_t mode;       // 当前关节模式
    uint8_t ReadBit;    // 电机控制参数修改     是否成功位
    int8_t Temp;        // 电机当前平均温度
    uint8_t MError;     // 电机错误 标识

    COMData32 Read;     // 读取的当前 电机 的控制数据
    int16_t T;          // 当前实际电机输出力矩       7 + 8 描述

    int16_t W;          // 当前实际电机速度（高速）   8 + 7 描述
    float LW;           // 当前实际电机速度（低速）

    int16_t W2;         // 当前实际关节速度（高速）   8 + 7 描述
    float LW2;          // 当前实际关节速度（低速）

    int16_t Acc;        // 电机转子加速度       15+0 描述  惯量较小
    int16_t OutAcc;     // 输出轴加速度         12+3 描述  惯量较大

    int32_t Pos;        // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t Pos2;       // 关节编码器位置(输出编码器)

    int16_t gyro[3];    // 电机驱动板6轴传感器数据
    int16_t acc[3];

    // 力传感器的数据
    int16_t Fgyro[3];
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp;      // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

    int16_t Force16;    // 力传感器高16位数据
    int8_t Force8;      // 力传感器低8位数据

    uint8_t FError;     // 足端传感器错误标识

    int8_t Res[1];      // 通讯保留字节

} ServoComdV3;          // 加上数据包的包头 和CRC 78字节（4+70+4）

typedef struct
{
    uint8_t head[2];    // 包头         	 2Byte
    RIS_Mode_t mode;    // 电机控制模式结构体 1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据结构体 11Byte
    uint16_t  CRC16;    // CRC          	 2Byte
} MotorData_t;          // 返回数据

typedef struct
{
    uint8_t none[8];            // 保留

} LowHzMotorCmd;

typedef struct
{                               // 以 4个字节一组排列 ，不然编译器会凑整
                                // 定义 数据
    uint8_t mode;               // 关节模式选择
    uint8_t ModifyBit;          // 电机控制参数修改位
    uint8_t ReadBit;            // 电机控制参数发送位
    uint8_t reserved;

    COMData32 Modify;           // 电机参数修改 的数据
    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    int16_t T;                  // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;                  // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos;                // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P;                // 关节刚度系数 x2048  4+11 描述
    int16_t K_W;                // 关节速度系数 x1024  5+10 描述

    uint8_t LowHzMotorCmdIndex; // 保留
    uint8_t LowHzMotorCmdByte;  // 保留

    COMData32 Res[1];           // 通讯 保留字节  用于实现别的一些通讯内容

} MasterComdV3; // 加上数据包的包头 和CRC 34字节

typedef struct
{
    // 定义 电机控制命令数据包
    uint8_t head[2];    // 包头                2Byte
    RIS_Mode_t mode;    // 电机控制模式结构体  1Byte
    RIS_Comd_t comd;    // 电机期望数据结构体 12Byte
    uint16_t   CRC16;   // CRC                 2Byte
} ControlData_t;     //电机控制命令数据包

#pragma pack()

typedef struct
{
	// 要发送的17字节控制电机的数据
	ControlData_t motor_send_data;   //电机控制数据结构体
	//------------------------------------------------------------------------------

	//上面的结构体的数据的赋值
	int hex_len;                        // 发送的16进制命令数组长度, 34
	long long send_time;                // 发送该命令的时间, 微秒(us)
	// 待发送的各项数据
	unsigned short id;                  // 电机ID，15代表全部电机
	unsigned short mode;                // 0:锁定, 1:闭环FOC, 2：编码器校准，3——7：保留
	//实际给FOC的指令力矩为：
	// K_P*delta_Pos + K_W*delta_W + T
	float T;                            // 期望关节的输出力矩（电机本身的力矩）（Nm）
	float W;                            // 期望关节速度（电机本身的速度）(rad/s)
	float Pos;                          // 期望关节位置（rad）
	float K_P;                          // 关节刚度系数
	float K_W;                          // 关节速度系数
	//保留
	COMData32 Res;                      // 通讯 保留字节  用于实现别的一些通讯内容
} MOTOR_send;

//接收数据结构体
typedef struct
{
	// 定义接收数据
	MotorData_t motor_recv_data;    //电机接收数据结构体，详见motor_msg.h

	int hex_len;                        // 接收的16进制命令数组长度, 78
	long long resv_time;                // 接收该命令的时间, 微秒(us)
	int correct;                        // 接收数据是否完整（1完整，0不完整）
	// 解读得出的电机数据
	unsigned char motor_id;             // 电机ID
	unsigned char mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制
	int Temp;                           // 温度
	unsigned char MError;               // 错误码
	float T;                            // 当前实际电机输出力矩
	float W;														// speed
	float Pos;                          // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
	float footForce;												// 足端气压传感器数据 12bit (0-4095)

} MOTOR_recv;


uint32_t crc32_core(uint32_t *ptr, uint32_t len);
int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);

#endif


