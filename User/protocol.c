#include "protocol.h"
uint8_t host_data_save(uint8_t host_order_type, uint8_t Res)
{
	switch(host_order_type)
	{
		case 1:
			return typeA_data(Res);
			break;
		case 2:
			return 0;
			break;
		default:
			break;
	}
	return 0;
}

uint8_t uint2float[4];	//uint8转float
uint8_t typeA_count = 0;	//收到的字节个数
uint8_t typeA_num = 0;	//状态编号
Chassis_Motion chassis_motion = {0};
float typeA_float = 0;
uint8_t typeA_data(uint8_t Res)
{
	if(typeA_count < 4)
	{
		uint2float[typeA_count] = Res;
		typeA_count++;
	}
	if(typeA_count == 4)	//每接收4个字节，转换1次float
	{
		typeA_float = *(float *)uint2float;
		typeA_count = 0;	//接收计数清零
		memset(uint2float, 0, sizeof(uint8_t)*4);	//数组清零
		switch(typeA_num)
		{
			case 0:
				chassis_motion.vx = typeA_float;
				typeA_num = 1;
				break;
			case 1:
				chassis_motion.vy = typeA_float;
				typeA_num = 2;
				break;
			case 2:
				chassis_motion.omega = typeA_float;
				typeA_num = 0;		
				return 0;
				break;
		}
	}
	return 1;
}

