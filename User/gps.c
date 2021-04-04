#include "gps.h"


void clrStruct(void)
{
	save_data.isGetData = false;
	save_data.isParseData = false;
	save_data.isUsefull = false;
	memset(save_data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
	memset(save_data.UTCTime, 0, UTCTime_Length);
	memset(save_data.latitude, 0, latitude_Length);
	memset(save_data.N_S, 0, N_S_Length);
	memset(save_data.longitude, 0, longitude_Length);
	memset(save_data.E_W, 0, E_W_Length);
}


void parseGpsBuffer(void)
{
	char *subString;
	char *subStringNext;
	char i = 0;
	if (save_data.isGetData)
	{
		save_data.isGetData = false;
		printf("**************\r\n");
		printf(save_data.GPS_Buffer);

		
		for (i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(save_data.GPS_Buffer, ",")) == NULL)
					errorLog(1);	//解析错误
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(save_data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
						case 3:memcpy(save_data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(save_data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(save_data.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(save_data.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					save_data.isParseData = true;
					if(usefullBuffer[0] == 'A')
						save_data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						save_data.isUsefull = false;

				}
				else
				{
					errorLog(2);	//解析错误
				}
			}


		}
	}
}


void printGpsBuffer(void)
{
	if (save_data.isParseData)
	{
		save_data.isParseData = false;
		
//		printf("save_data.UTCTime = ");
//		printf(save_data.UTCTime);
//		printf("\r\n");

		if(save_data.isUsefull)
		{
			save_data.isUsefull = false;
			printf("save_data.latitude = ");
			printf(save_data.latitude);
			printf("\r\n");


			printf("save_data.N_S = ");
			printf(save_data.N_S);
			printf("\r\n");

			printf("save_data.longitude = ");
			printf(save_data.longitude);
			printf("\r\n");

			printf("save_data.E_W = ");
			printf(save_data.E_W);
			printf("\r\n");
		}
		else
		{
			printf("GPS DATA is not usefull!\r\n");
		}
		
	}
}

void errorLog(int code)
{
	printf("error code:%d\n", code);
}

