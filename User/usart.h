#ifndef __USART_H
#define __USART_H
#include "stm32f4xx_hal.h"
#include "stdio.h"	
#include "string.h"
#include "gps.h"
#include "main.h"
#ifdef FREERTOS
#include "freertos.h"
#endif
#include "protocol.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART_RX 			1		//使能（1）/禁止（0）串口1接收
#define false 0
#define true 1
	
extern char  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;


extern char rxdatabufer;
extern uint16_t point1;
extern GPS_Data save_data;
extern uint8_t GpsTempChar;
extern uint8_t HostTempChar;
extern uint8_t DebugTempChar;
extern uint32_t host_heartbeat;

extern uint8_t debug_state;

int fputc(int ch, FILE *f);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);          	//串口1中断服务程序
void CLR_Buf(void);
uint8_t Hand(char *a);

#endif


