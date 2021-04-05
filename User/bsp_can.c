/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "cmsis_os.h"


//moto_measure_t moto_pit;
//moto_measure_t moto_yaw;
//moto_measure_t moto_poke;	//拨单电机


void get_total_angle(moto_measure_t *p);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		CAN_FilterConfigStructure;

	CAN_FilterConfigStructure.FilterBank = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}


}

/*******************************************************************************************
  * @Func			void can_filter_recv_special(CAN_HandleTypeDef* _hcan, s16 id)
  * @Brief    待测试！！！
  * @Param		只接收filtered id，其他的全屏蔽。
  * @Retval		eg： 	CAN1_FilterConfiguration(0, HOST_CONTROL_ID);
										CAN1_FilterConfiguration(1, SET_CURRENT_ID);
										CAN1_FilterConfiguration(2, SET_VOLTAGE_ID);
										CAN1_FilterConfiguration(3, ESC_CAN_DEVICE_ID);
										CAN1_FilterConfiguration(4, SET_POWER_ID);
										CAN1_FilterConfiguration(5, SET_LIMIT_RECOVER_ID);
  * @Date     2016年11月11日
 *******************************************************************************************/
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id)
{
	CAN_FilterTypeDef   cf;
	cf.FilterBank = filter_number;	//过滤器组编号
	cf.FilterMode = CAN_FILTERMODE_IDMASK;	//id屏蔽模式
	cf.FilterScale = CAN_FILTERSCALE_32BIT;	//32bit 滤波
	cf.FilterIdHigh = (filtered_id<<21) >> 16;	//high 16 bit		其实这两个结构体成员变量是16位宽
	cf.FilterIdLow = filtered_id<<21;	//low 16bit
	cf.FilterMaskIdHigh = 0xFFFF;
	cf.FilterMaskIdLow = 0xFFF8;	//IDE[2], RTR[1] TXRQ[0] 低三位不考虑。
	cf.FilterFIFOAssignment = CAN_FilterFIFO0;
	cf.SlaveStartFilterBank = 14;	//can1(0-13)和can2(14-27)分别得到一半的filter
	cf.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &cf);
}


HAL_StatusTypeDef can_send_msg()
{
//	if(_hcan->Instance->ESR){
//		//can error occured, sleep can and reset!
//		_hcan->Instance->MCR |= 0x02;
//		_hcan->Instance->MCR &= ~(0x02);
//	}//这个是zw试过的可以解决can错误  有待验证！
	return HAL_OK;
}

float ZGyroModuleAngle;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN Receive
{
	if(hcan == my_hcan1->hcan)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, my_hcan1->pRxMsg, my_hcan1->RxData);
		HAL_CAN_ActivateNotification(my_hcan1->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		switch(my_hcan1->pRxMsg->StdId)
		{
			case CAN_3510Moto1_ID:
			case CAN_3510Moto2_ID:
			case CAN_3510Moto3_ID:
			case CAN_3510Moto4_ID:
			{
				static u8 i;
				i = my_hcan1->pRxMsg->StdId - CAN_3510Moto1_ID;
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], my_hcan1) : get_moto_measure(&moto_chassis[i], my_hcan1);
				get_moto_measure(&moto_info, my_hcan1);
				//get_moto_measure(&moto_chassis[i], _hcan);
			}
			break;
			case CAN_3510Moto5_ID:
			case CAN_3510Moto6_ID:
			{
				static u8 j;
				j = my_hcan1->pRxMsg->StdId - CAN_3510Moto5_ID;
				moto_brush[j].msg_cnt++ <= 50	?	get_moto_offset(&moto_brush[j], my_hcan1) : get_moto_measure(&moto_brush[j], my_hcan1);
				get_moto_measure(&moto_info, my_hcan1);
			}
			break;
		}
	}
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, My_CAN_HandleTypeDef *hcan)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;
	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->RxData[0]<<8 | hcan->RxData[1]) ;
	ptr->real_current  = (int16_t)(hcan->RxData[2]<<8 | hcan->RxData[3]);
	ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
	ptr->given_current = (int16_t)(hcan->RxData[4]<<8 | hcan->RxData[5])/-5;
	ptr->hall = hcan->RxData[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, My_CAN_HandleTypeDef *hcan)
{
	ptr->angle = (uint16_t)(hcan->RxData[0]<<8 | hcan->RxData[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_1to4_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
	if(hcan == my_hcan1->hcan)
	{
		my_hcan1->pTxMsg->StdId = 0x200;
		my_hcan1->pTxMsg->IDE = CAN_ID_STD;
		my_hcan1->pTxMsg->RTR = CAN_RTR_DATA;
		my_hcan1->pTxMsg->DLC = 0x08;
		my_hcan1->pTxMsg->TransmitGlobalTime = DISABLE;
		
		my_hcan1->TxData[0] = iq1 >> 8;
		my_hcan1->TxData[1] = iq1;
		my_hcan1->TxData[2] = iq2 >> 8;
		my_hcan1->TxData[3] = iq2;
		my_hcan1->TxData[4] = iq3 >> 8;
		my_hcan1->TxData[5] = iq3;
		my_hcan1->TxData[6] = iq4 >> 8;
		my_hcan1->TxData[7] = iq4;
		
		HAL_CAN_AddTxMessage(my_hcan1->hcan, my_hcan1->pTxMsg, my_hcan1->TxData, &my_hcan1->TxMailbox);
	}
}

void set_moto_5to8_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
	if(hcan == my_hcan1->hcan)
	{
		my_hcan1->pTxMsg->StdId = 0x1FF;
		my_hcan1->pTxMsg->IDE = CAN_ID_STD;
		my_hcan1->pTxMsg->RTR = CAN_RTR_DATA;
		my_hcan1->pTxMsg->DLC = 0x08;
		my_hcan1->pTxMsg->TransmitGlobalTime = DISABLE;
		
		my_hcan1->TxData[0] = iq1 >> 8;
		my_hcan1->TxData[1] = iq1;
		my_hcan1->TxData[2] = iq2 >> 8;
		my_hcan1->TxData[3] = iq2;
		my_hcan1->TxData[4] = iq3 >> 8;
		my_hcan1->TxData[5] = iq3;
		my_hcan1->TxData[6] = iq4 >> 8;
		my_hcan1->TxData[7] = iq4;
		
		HAL_CAN_AddTxMessage(my_hcan1->hcan, my_hcan1->pTxMsg, my_hcan1->TxData, &my_hcan1->TxMailbox);
	}
}



