#include "usart.h"	
char rxdatabufer;
uint16_t point1 = 0;
uint8_t host_rx_state = 0;
uint8_t host_order_type = 0;

//重定义fputc函数, uart8作为DEBUG口
int fputc(int ch, FILE *f)
{      
	while((huart8.Instance->SR&0X40)==0);//循环发送,直到发送完毕   
    huart8.Instance->DR = (uint8_t) ch;
	return ch;
}
 
#if EN_USART_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       //接收状态标记	  

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)           	//串口1中断服务程序
{
	BaseType_t xHigherPriorityTaskWoken;
#ifdef FREERTOS
	xHigherPriorityTaskWoken = pdFALSE;
#endif
	uint8_t Res = 0;
	if(huart == &huart6) 
	{
		Res = GpsTempChar;
		if(Res == '$')
		{
			point1 = 0;	
		}
		
		USART_RX_BUF[point1++] = Res;
		
		if(USART_RX_BUF[0] == '$' && USART_RX_BUF[4] == 'M' && USART_RX_BUF[5] == 'C')			//确定是否收到"GPRMC/GNRMC"这一帧数据
		{
			if(Res == '\n')									   
			{
				memset(save_data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
				memcpy(save_data.GPS_Buffer, USART_RX_BUF, point1); 	//保存数据
				save_data.isGetData = true;
				point1 = 0;
				memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空				
			}		
		}
		
		if(point1 >= USART_REC_LEN)
		{
			point1 = USART_REC_LEN;
		}		
   }
	if(huart == &huart7)
	{
		Res = HostTempChar;
		uint8_t data_processing = 0;
		switch(host_rx_state)
		{
			case 0:
				if(Res == 0xFF)
				{
					host_rx_state = 1;
				}
				else
				{
					host_rx_state = 0;
				}
				break;
			case 1:
				host_order_type = Res;
				if(host_order_type == 1 || host_order_type == 2)
				{
					host_rx_state = 2;
				}
				else
				{
					host_rx_state = 0;
				}
				break;
			case 2:
				data_processing = host_data_save(host_order_type, Res);
				if(data_processing == 0)
				{
					host_rx_state = 0;
				}
				else
				{
					host_rx_state = 2;
				}
				break;
			default :
				break;
		}
	}
#ifdef FREERTOS
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}


uint8_t Hand(char *a)                   // 串口命令识别函数
{ 
    if(strstr(USART_RX_BUF,a)!=NULL)
	    return 1;
	else
		return 0;
}

void CLR_Buf(void)                           // 串口缓存清理
{
	memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空
  point1 = 0;                    
}

#endif	

