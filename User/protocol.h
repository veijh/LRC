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
uint8_t host_data_save(uint8_t host_order_type, uint8_t Res);
uint8_t typeA_data(uint8_t Res);
extern Chassis_Motion chassis_motion;
#endif
