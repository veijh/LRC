#include "protocol.h"
uint8_t host_data_save(uint8_t host_order_type, uint8_t Res)
{
	uint8_t is_processing = 0;
	switch(host_order_type)
	{
		case 1:
			is_processing = typeA_data(Res);
			break;
		case 2:
			is_processing = typeB_data(Res);
			break;
		case 3:
			is_processing = typeC_data(Res);
			break;
		default:
			is_processing = 0;
			break;
	}
	return is_processing;
}

Type_Data typeA = {0};
Chassis_Motion chassis_motion = {0};
uint8_t typeA_data(uint8_t Res)
{
	if(typeA.count < 4)
	{
		typeA.uint2float[typeA.count] = Res;
		typeA.count++;
	}
	if(typeA.count == 4)	//每接收4个字节，转换1次float
	{
		typeA.float_data = *(float *)typeA.uint2float;
		typeA.count = 0;	//接收计数清零
		memset(typeA.uint2float, 0, sizeof(uint8_t)*4);	//数组清零
		switch(typeA.num)
		{
			case 0:
				chassis_motion.vx = typeA.float_data;
				typeA.num = 1;
				break;
			case 1:
				chassis_motion.vy = typeA.float_data;
				typeA.num = 2;
				break;
			case 2:
				chassis_motion.omega = typeA.float_data;
				typeA.num = 0;		
				return 0;
				break;
		}
	}
	return 1;
}

Type_Data typeB = {0};
float brush_spd_from_host = 0.0f;
uint8_t typeB_data(uint8_t Res)
{
	if(typeB.count < 4)
	{
		typeB.uint2float[typeB.count] = Res;
		typeB.count++;
	}
	if(typeB.count == 4)	//每接收4个字节，转换1次float
	{
		typeB.float_data = *(float *)typeB.uint2float;
		typeB.count = 0;	//接收计数清零
		memset(typeB.uint2float, 0, sizeof(uint8_t)*4);	//数组清零
		switch(typeB.num)
		{
			case 0:
				brush_spd_from_host = typeB.float_data;
				typeB.num = 0;
				return 0;
				break;
		}
	}
	return 1;
}

Type_Data typeC = {0};
float fan_spd_from_host = 0.0f;
uint8_t typeC_data(uint8_t Res)
{
	if(typeC.count < 4)
	{
		typeC.uint2float[typeC.count] = Res;
		typeC.count++;
	}
	if(typeC.count == 4)	//每接收4个字节，转换1次float
	{
		typeC.float_data = *(float *)typeC.uint2float;
		typeC.count = 0;	//接收计数清零
		memset(typeC.uint2float, 0, sizeof(uint8_t)*4);	//数组清零
		switch(typeC.num)
		{
			case 0:
				fan_spd_from_host = typeC.float_data;
				typeC.num = 0;
				return 0;
				break;
		}
	}
	return 1;
}

