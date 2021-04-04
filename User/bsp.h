#ifndef __BSP_H
#define __BSP_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "bsp_can.h"
#include "usart.h"
#include "gps.h"
#include "bsp_imu.h"
#ifdef FREERTOS
#include "freertos.h"
#endif
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
void bsp_init(void);
float get_temprate(void);
#endif
