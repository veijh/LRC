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

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
#include "mytype.h"
#include "stm32f4xx_hal_can.h"

/*CAN发送或是接收的ID*/
typedef enum
{

	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_3510Moto5_ID = 0x205,
	CAN_3510Moto6_ID = 0x206,
	CAN_3510Moto7_ID = 0x207,
	CAN_3510Moto8_ID = 0x208,
	
}CAN_Message_ID;

#define FILTER_BUF_LEN		5
/*接收到的电机参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
	int16_t		speed_rpm_filtered;	//滤波后的速度
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;				//temp
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;			//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			spd_buf_idx;				//滤波数组索引
	int32_t		spd_buf[FILTER_BUF_LEN];	//滤波数组
	int32_t		spd_sum;
	u16			fited_angle;
	u32			msg_cnt;
	u8			state;
	uint32_t 	heartbeat;
}moto_measure_t;

typedef struct{
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef *pTxMsg;
	CAN_RxHeaderTypeDef *pRxMsg;
	uint8_t TxData[8];
	uint8_t RxData[8];
	uint32_t TxMailbox;
}My_CAN_HandleTypeDef;


/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_chassis[];
extern moto_measure_t  moto_brush[];
extern moto_measure_t  moto_yaw,moto_pit,moto_poke,moto_info;
extern float real_current_from_judgesys; //unit :mA
extern float dynamic_limit_current;	//unit :mA,;	//from judge_sys
extern My_CAN_HandleTypeDef *my_hcan1; 


void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr, My_CAN_HandleTypeDef *hcan);
void get_moto_offset(moto_measure_t *ptr, My_CAN_HandleTypeDef *hcan);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_1to4_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto_5to8_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
#endif
