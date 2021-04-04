#ifndef __GPS_H
#define __GPS_H
#include "string.h"
#include "stdio.h"
#define false 0
#define true 1

#define GPS_Buffer_Length 80
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 


typedef struct 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//定位信息是否有效
}GPS_Data;

extern GPS_Data save_data;

void clrStruct(void);
void parseGpsBuffer(void);
void printGpsBuffer(void);
void errorLog(int code);

#endif
