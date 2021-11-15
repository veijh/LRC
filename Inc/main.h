/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T2DEBUG_Pin GPIO_PIN_1
#define T2DEBUG_GPIO_Port GPIOE
#define R2DEBUG_Pin GPIO_PIN_0
#define R2DEBUG_GPIO_Port GPIOE
#define HEAT_PWM_Pin GPIO_PIN_5
#define HEAT_PWM_GPIO_Port GPIOB
#define T2GPS_Pin GPIO_PIN_14
#define T2GPS_GPIO_Port GPIOG
#define HEAT_Pin GPIO_PIN_4
#define HEAT_GPIO_Port GPIOB
#define R2GPS_Pin GPIO_PIN_9
#define R2GPS_GPIO_Port GPIOG
#define J20_Pin GPIO_PIN_2
#define J20_GPIO_Port GPIOH
#define J22_Pin GPIO_PIN_3
#define J22_GPIO_Port GPIOH
#define J19_Pin GPIO_PIN_4
#define J19_GPIO_Port GPIOH
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOG
#define J21_Pin GPIO_PIN_5
#define J21_GPIO_Port GPIOH
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOG
#define NSS_Pin GPIO_PIN_6
#define NSS_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG
#define FANB_Pin GPIO_PIN_13
#define FANB_GPIO_Port GPIOD
#define FANA_Pin GPIO_PIN_12
#define FANA_GPIO_Port GPIOD
#define T2HOST_Pin GPIO_PIN_8
#define T2HOST_GPIO_Port GPIOE
#define R2HOST_Pin GPIO_PIN_7
#define R2HOST_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define FREERTOS
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
