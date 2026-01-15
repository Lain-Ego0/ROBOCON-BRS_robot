/**
 * @file ris_protocol.h
 * @author Yichao Zhang (unitree@qq.com)
 * @brief Go-M8010-6关节电机驱动 精简通讯指令集
 * @version 0.1
 * @date 2022-03-04
 * 
 * @copyright Copyright (c) unitree robotics .co.ltd. 2022
 */

#ifndef __RIS_PROTOCOL_H
#define __RIS_PROTOCOL_H

#include <stdint.h>

#pragma pack(1)

/**
* @brief 发送与接收的电机模式控制信息
 * 
 */
typedef struct
{		
	//位域操作：把uint8_t   8位分为4；3；1；三份
    uint8_t id     :4;      // 电机ID: 0,1...,13,14//--15表示向所有电机广播数据(此时无返回)
    uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none   :1;      // 保留位
} RIS_Mode_t;   			// 控制模式 1Byte

/**
 * @brief 期望关节输出的电机控制信息
 * 
 */
typedef struct
{
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des;        // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;          // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;          // 期望关节阻尼系数 unit: -1.0-1.0 (q15)    
} RIS_Comd_t;   			// 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 * 
 */
typedef struct
{
    int16_t  torque;        // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t  speed;         // 实际关节输出速度 unit: rad/s   (q8)
    int32_t  pos;           // 实际关节输出位置 unit: rad     (q15)
    int8_t   temp;          // 电机温度: -128~127°C
    uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force  :12;    // 足端气压传感器数据 12bit (0-4095)
    uint8_t  none   :1;     // 保留位
} RIS_Fbk_t;   				// 状态数据 11Byte


#pragma pack()

#endif

/* 
 * Actuator Communication Reduced Instruction Set
 * Unitree robotics (c) .Co.Ltd. 2022 All Rights Reserved.
 */
