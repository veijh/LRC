#include "bsp.h"
My_CAN_HandleTypeDef *my_hcan1;
GPS_Data save_data;
uint8_t GpsTempChar;
uint8_t HostTempChar;
uint8_t DebugTempChar;
void bsp_init(void)
{
	HAL_Delay(3500);
	//CAN
	#ifdef FREERTOS
	my_hcan1 = (My_CAN_HandleTypeDef *)pvPortMalloc(sizeof(My_CAN_HandleTypeDef));
	my_hcan1->hcan = &hcan1;
	my_hcan1->pRxMsg = (CAN_RxHeaderTypeDef *)pvPortMalloc(sizeof(CAN_RxHeaderTypeDef));
	my_hcan1->pTxMsg = (CAN_TxHeaderTypeDef *)pvPortMalloc(sizeof(CAN_TxHeaderTypeDef));
	#else
	
	#endif
	my_can_filter_init_recv_all(my_hcan1->hcan);
	HAL_CAN_Start(my_hcan1->hcan);
	HAL_CAN_ActivateNotification(my_hcan1->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	
	
	//UART
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);	//GPS
    HAL_UART_Receive_IT(&huart6, &GpsTempChar, 1);
	
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);	//HOST
    HAL_UART_Receive_IT(&huart7, &HostTempChar, 1);
	
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);	//DEBUG
    HAL_UART_Receive_IT(&huart8, &DebugTempChar, 1);
	
	//GPS
	clrStruct();
	
	//IMU
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//启动加热
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,0);
	mpu_device_init();
	init_quaternion();
	
	//风机
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,0);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,0);
}

float get_temprate(void)
{
    uint32_t adcx = 0;
    float temperate = 50;
	HAL_ADC_Start(&hadc1);
   	HAL_ADC_PollForConversion(&hadc1, 50);
 	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
  	{
		adcx = HAL_ADC_GetValue(&hadc1);
		temperate = (float)adcx * (3.3f / 4096.0f);
		temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
	}
    return temperate;
}
