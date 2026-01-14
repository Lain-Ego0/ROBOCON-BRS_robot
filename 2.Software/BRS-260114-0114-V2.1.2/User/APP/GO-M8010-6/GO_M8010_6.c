#include "GO_M8010_6.h"
#include "crc_ccitt.h"
#include <stdio.h>

//限制上下限//
#define SATURATE(_IN, _MIN, _MAX) {		\
 if (_IN < _MIN)						\
 _IN = _MIN;							\
 else if (_IN > _MAX)					\
 _IN = _MAX;							\
 } 

 //发送结构体数据赋值，把结构体的中的值，赋给结构体的中的控制结构体
int modify_data(MOTOR_send *motor_s)
{
	//数据长度
	motor_s->hex_len = 17;
	motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	//限制范围上下限
	//SATURATE(motor_s->id,   0,    15);
	//SATURATE(motor_s->mode, 0,    7);
	SATURATE(motor_s->K_P,  0.0f,   25.599f);
	SATURATE(motor_s->K_W,  0.0f,   25.599f);
	SATURATE(motor_s->T,   -127.99f,  127.99f);
	SATURATE(motor_s->W,   -804.00f,  804.00f);
	SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

	//发送数据赋值
	motor_s->motor_send_data.mode.id   = motor_s->id;
	motor_s->motor_send_data.mode.status  = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/838860.8f;//25.6f*32768;
	motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/838860.8f;//25.6f*32768;
	motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/205887.89f;//6.2832f*32768;
	motor_s->motor_send_data.comd.spd_des  = motor_s->W/1608.4992f;//6.2832f*256;
	motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
	return 0;
}

 //接收结构体数据赋值
int extract_data(MOTOR_recv *motor_r)
{
	//CRC校验码核对-----判断是否读取数据---
	if(motor_r->motor_recv_data.CRC16 !=
	crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
	{
		//printf("[WARNING] Receive data CRC error");
		motor_r->correct = 0;
		return motor_r->correct;
	} else {
		//printf("111111111111111111111111");
		//保存数据结构体  = 接收数据结构体
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		//转速偏差
		motor_r->W = -0.14137188f+((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
		motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		return motor_r->correct;
	}
}

