#ifndef __BSP_H
#define __BSP_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "bsp_can.h"
#ifdef FREERTOS
#include "freertos.h"
#endif
extern CAN_HandleTypeDef hcan1;
void bsp_init(void);

#endif
