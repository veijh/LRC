#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "stm32f4xx_hal.h"
#include "string.h"
typedef struct
{
	float vx;
	float vy;
	float omega;
}Chassis_Motion;

typedef struct
{
	uint8_t uint2float[4];	//uint8转float
	uint8_t count;	//收到的字节个数
	uint8_t num;	//状态编号
	float float_data;
}Type_Data;

uint8_t host_data_save(uint8_t host_order_type, uint8_t Res);
uint8_t typeA_data(uint8_t Res);
uint8_t typeB_data(uint8_t Res);
uint8_t typeC_data(uint8_t Res);
extern Chassis_Motion chassis_motion;
extern float brush_spd_from_host;
extern float fan_spd_from_host;
#endif
