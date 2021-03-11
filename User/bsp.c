#include "bsp.h"
My_CAN_HandleTypeDef *my_hcan1;
void bsp_init(void)
{
	HAL_Delay(1500);
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
}

