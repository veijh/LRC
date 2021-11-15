/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "pid.h"
#include "gps.h"
#include "usart.h"
#include "protocol.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myGetGPS */
osThreadId_t myGetGPSHandle;
const osThreadAttr_t myGetGPS_attributes = {
  .name = "myGetGPS",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for myMotorControl */
osThreadId_t myMotorControlHandle;
const osThreadAttr_t myMotorControl_attributes = {
  .name = "myMotorControl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myGetIMU */
osThreadId_t myGetIMUHandle;
const osThreadAttr_t myGetIMU_attributes = {
  .name = "myGetIMU",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for myCtrlIMUTemp */
osThreadId_t myCtrlIMUTempHandle;
const osThreadAttr_t myCtrlIMUTemp_attributes = {
  .name = "myCtrlIMUTemp",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myChassisCtrl */
osThreadId_t myChassisCtrlHandle;
const osThreadAttr_t myChassisCtrl_attributes = {
  .name = "myChassisCtrl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myBrushCtrl */
osThreadId_t myBrushCtrlHandle;
const osThreadAttr_t myBrushCtrl_attributes = {
  .name = "myBrushCtrl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myLED */
osThreadId_t myLEDHandle;
const osThreadAttr_t myLED_attributes = {
  .name = "myLED",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for myMtHrtBt */
osThreadId_t myMtHrtBtHandle;
const osThreadAttr_t myMtHrtBt_attributes = {
  .name = "myMtHrtBt",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for myFanCtrl */
osThreadId_t myFanCtrlHandle;
const osThreadAttr_t myFanCtrl_attributes = {
  .name = "myFanCtrl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
extern My_CAN_HandleTypeDef *my_hcan1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART8_Init(void);
static void MX_SPI5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART7_Init(void);
void StartDefaultTask(void *argument);
void StartGetGPS(void *argument);
void StartMotorControl(void *argument);
void StartGetIMU(void *argument);
void StartCtrlIMUTemp(void *argument);
void StartChassisCtrl(void *argument);
void StartBrushCtrl(void *argument);
void StartLED(void *argument);
void StartMtHrtBt(void *argument);
void StartFanCtrl(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_UART8_Init();
  MX_SPI5_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  bsp_init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myGetGPS */
  myGetGPSHandle = osThreadNew(StartGetGPS, NULL, &myGetGPS_attributes);

  /* creation of myMotorControl */
  myMotorControlHandle = osThreadNew(StartMotorControl, NULL, &myMotorControl_attributes);

  /* creation of myGetIMU */
  myGetIMUHandle = osThreadNew(StartGetIMU, NULL, &myGetIMU_attributes);

  /* creation of myCtrlIMUTemp */
  myCtrlIMUTempHandle = osThreadNew(StartCtrlIMUTemp, NULL, &myCtrlIMUTemp_attributes);

  /* creation of myChassisCtrl */
  myChassisCtrlHandle = osThreadNew(StartChassisCtrl, NULL, &myChassisCtrl_attributes);

  /* creation of myBrushCtrl */
  myBrushCtrlHandle = osThreadNew(StartBrushCtrl, NULL, &myBrushCtrl_attributes);

  /* creation of myLED */
  myLEDHandle = osThreadNew(StartLED, NULL, &myLED_attributes);

  /* creation of myMtHrtBt */
  myMtHrtBtHandle = osThreadNew(StartMtHrtBt, NULL, &myMtHrtBt_attributes);

  /* creation of myFanCtrl */
  myFanCtrlHandle = osThreadNew(StartFanCtrl, NULL, &myFanCtrl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEAT_GPIO_Port, HEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, J20_Pin|J22_Pin|J19_Pin|J21_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEAT_Pin */
  GPIO_InitStruct.Pin = HEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J20_Pin J22_Pin J19_Pin J21_Pin */
  GPIO_InitStruct.Pin = J20_Pin|J22_Pin|J19_Pin|J21_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED8_Pin LED7_Pin LED6_Pin LED5_Pin
                           LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGetGPS */
/**
* @brief Function implementing the myGetGPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetGPS */
void StartGetGPS(void *argument)
{
  /* USER CODE BEGIN StartGetGPS */
  /* Infinite loop */
  for(;;)
  {
	parseGpsBuffer();
	//printGpsBuffer();
    osDelay(100);
  }
  /* USER CODE END StartGetGPS */
}

/* USER CODE BEGIN Header_StartMotorControl */
/**
* @brief Function implementing the myMotorControl thread.
* @param argument: Not used
* @retval None
*/
#define Pi 3.1415926f
#define L (43.6f + 36.5f) //	Length/2 + Width/2
#define R 6.0f	// 轮子半径 cm
#define RATIO 19	// 3508_ratio = 19
#define WHEEL_KP 2.0f
#define WHEEL_KI 0.075f
#define WHEEL_KD 0.0f
#define WHEEL_DV 45.6f
float vx = 0.0f, vy = 0.0f, omega = 0.0f; // cm/s rad/s
moto_measure_t moto_chassis[4] = {0};// 4 chassis moto
float set_wheel_spd[4] = {0, 0, 0, 0};	//LF RF RB LB
float rt_wheel_spd[4] = {0, 0, 0,0};
/* USER CODE END Header_StartMotorControl */
void StartMotorControl(void *argument)
{
  /* USER CODE BEGIN StartMotorControl */
  for(int i=0; i<4; i++)
  {
	PID_struct_init(&pid_wheel_spd[i], POSITION_PID, 16000, 16000,
								WHEEL_KP,	WHEEL_KI,	WHEEL_KD	);  //4 motos angular rate closeloop.
  }
  /* Infinite loop */
  for(;;)
  {
	//主机掉线在chassis_ctrl任务中处理
	set_wheel_spd[0] = ((+vx	+vy	+omega*L)/R * (30/Pi)) * RATIO;
	set_wheel_spd[1] = ((-vx	+vy	+omega*L)/R * (30/Pi)) * RATIO;
	set_wheel_spd[2] = ((-vx	-vy	+omega*L)/R * (30/Pi)) * RATIO;
	set_wheel_spd[3] = ((+vx	-vy	+omega*L)/R * (30/Pi)) * RATIO;
	for(int i=0; i<4; i++)
	{
		//在电机确认上线后，才能PID调速
		if(moto_chassis[i].state == 1)
		{
			if(fabs(set_wheel_spd[i]-rt_wheel_spd[i])>WHEEL_DV)
			{
				rt_wheel_spd[i] += WHEEL_DV * (set_wheel_spd[i]>rt_wheel_spd[i] ? 1.0f:-1.0f);
			}
			else
			{
				rt_wheel_spd[i] = set_wheel_spd[i];
			}
			pid_calc(&pid_wheel_spd[i], moto_chassis[i].speed_rpm, rt_wheel_spd[i]);
		}
		else
		{
			pid_output_reset(&pid_wheel_spd[i]);
		}
	}
	set_moto_1to4_current(my_hcan1->hcan, 
						  pid_wheel_spd[0].pos_out, 
						  pid_wheel_spd[1].pos_out,
						  pid_wheel_spd[2].pos_out,
						  pid_wheel_spd[3].pos_out);
	osDelay(10);
  }
  /* USER CODE END StartMotorControl */
}

/* USER CODE BEGIN Header_StartGetIMU */
/**
* @brief Function implementing the myGetIMU thread.
* @param argument: Not used
* @retval None
*/
extern float set_imu_temp;
extern imu_t imu;
/* USER CODE END Header_StartGetIMU */
void StartGetIMU(void *argument)
{
  /* USER CODE BEGIN StartGetIMU */
  /* Infinite loop */
  for(;;)
  {
	mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update();
//	printf("%f,%f,%f\n",imu.pit, imu.yaw, imu.rol);
    osDelay(1);
  }
  /* USER CODE END StartGetIMU */
}

/* USER CODE BEGIN Header_StartCtrlIMUTemp */
/**
* @brief Function implementing the myCtrlIMUTemp thread.
* @param argument: Not used
* @retval None
*/
extern pid_t pid_imu_tmp;
float set_imu_temp = 50.0f;
int imu_temp_ctrl = 1;
/* USER CODE END Header_StartCtrlIMUTemp */
void StartCtrlIMUTemp(void *argument)
{
  /* USER CODE BEGIN StartCtrlIMUTemp */
  PID_struct_init(&pid_imu_tmp, POSITION_PID, 4500, 4500,
								300.0f,	10.0f, 0.0f	);  //4 motos angular rate closeloop.
  /* Infinite loop */
  for(;;)
  {
	if(imu_temp_ctrl)
	{
		if(imu.temp < set_imu_temp)
		{
			pid_calc(&pid_imu_tmp, imu.temp, set_imu_temp);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pid_imu_tmp.pos_out);
		}
		else
		{
			pid_output_reset(&pid_imu_tmp);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		}
		//printf("%f\n",imu.temp);
	}
	osDelay(1);
  }
  /* USER CODE END StartCtrlIMUTemp */
}

/* USER CODE BEGIN Header_StartChassisCtrl */
/**
* @brief Function implementing the myChassisCtrl thread.
* @param argument: Not used
* @retval None
*/
extern Chassis_Motion chassis_motion;
extern float brush_spd_from_host;
extern uint8_t host_state;
uint8_t debug_state = 0;
extern float set_brush_spd[2];
extern float fan_spd_from_host;
extern uint32_t fan_spd;
/* USER CODE END Header_StartChassisCtrl */
void StartChassisCtrl(void *argument)
{
  /* USER CODE BEGIN StartChassisCtrl */
  /* Infinite loop */
  for(;;)
  {
	if(host_state == 1)	//主机上线
	{
		vx = chassis_motion.vx;
		vy = chassis_motion.vy;
		omega = chassis_motion.omega;
		set_brush_spd[0] = brush_spd_from_host*RATIO;
		set_brush_spd[1] = -brush_spd_from_host*RATIO;
		if(fan_spd_from_host < 0.0f)
		{
			fan_spd = 0;
		}
		else if(fan_spd_from_host < 999.0f)
		{
			fan_spd = (uint32_t)fan_spd_from_host;
		}
		else
		{
			fan_spd = 999;
		}
//		fan_spd = (uint32_t)fan_spd_from_host;
	}
	else if(debug_state == 1)
	{
		vx = 0.0f;
		vy = 0.0f;
		omega = 0.0f;
		set_brush_spd[0] = RATIO*120.0f;
		set_brush_spd[1] = -RATIO*120.0f;
		fan_spd = 799;
	}
	else
	{
		vx = 0.0f;
		vy = 0.0f;
		omega = 0.0f;
		set_brush_spd[0] = 0.0f;
		set_brush_spd[1] = 0.0f;
		fan_spd = 0;
	}
	osDelay(10);
  }
  /* USER CODE END StartChassisCtrl */
}

/* USER CODE BEGIN Header_StartBrushCtrl */
/**
* @brief Function implementing the myBrushCtrl thread.
* @param argument: Not used
* @retval None
*/
#define BRUSH_DV 45.6f
#define BRUSH_DDV 9.12f
moto_measure_t moto_brush[2] = {0};
float set_brush_spd[2] = {19*2*60.0, -19*2*60.0};
float rt_brush_spd[2] = {0, 0};

/* USER CODE END Header_StartBrushCtrl */
void StartBrushCtrl(void *argument)
{
  /* USER CODE BEGIN StartBrushCtrl */
	PID_struct_init(&pid_brush_spd[0], POSITION_PID, 16000, 16000,
								2.0f,	0.075f,	0.0f	); 
	pid_brush_spd[0].integral_seperation = 1;
	pid_brush_spd[0].integral_threshold = 500;
	PID_struct_init(&pid_brush_spd[1], POSITION_PID, 16000, 16000,
								3.5f,	0.32f,	0.0f	); 
	pid_brush_spd[1].integral_seperation = 1;
	pid_brush_spd[1].integral_threshold = 500;
  /* Infinite loop */
  for(;;)
  {
	for(int i=0; i<2; i++)
	{
		if(moto_brush[i].state == 1)
		{
			if(fabs(set_brush_spd[i]-rt_brush_spd[i])>BRUSH_DV)
			{
				rt_brush_spd[i] += BRUSH_DV * (set_brush_spd[i]>rt_brush_spd[i] ? 1.0f:-1.0f);
			}
			else
			{
				rt_brush_spd[i] = set_brush_spd[i];
			}
			pid_calc(&pid_brush_spd[i], moto_brush[i].speed_rpm_filtered, rt_brush_spd[i]);
		}
		else	//电机掉线
		{
			pid_output_reset(&pid_brush_spd[i]);
		}
	}
	printf("%d,%f\n", moto_brush[0].speed_rpm_filtered, rt_brush_spd[0]);
	set_moto_5to8_current(my_hcan1->hcan, 
						  pid_brush_spd[0].pos_out, 
						  pid_brush_spd[1].pos_out,
						  0,
						  0);
	osDelay(10);
  }
  /* USER CODE END StartBrushCtrl */
}

/* USER CODE BEGIN Header_StartLED */
/**
* @brief Function implementing the myLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED */
void StartLED(void *argument)
{
  /* USER CODE BEGIN StartLED */
  /* Infinite loop */
  for(;;)
  {
	  if(debug_state == 1)
	  {
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin, GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin, GPIO_PIN_SET);
	  }
    osDelay(1000);
  }
  /* USER CODE END StartLED */
}

/* USER CODE BEGIN Header_StartMtHrtBt */
/**
* @brief Function implementing the myMtHrtBt thread.
* @param argument: Not used
* @retval None
*/
extern uint32_t host_heartbeat;
uint8_t host_state = 0;
/* USER CODE END Header_StartMtHrtBt */
void StartMtHrtBt(void *argument)
{
  /* USER CODE BEGIN StartMtHrtBt */
  /* Infinite loop */
  for(;;)
  {
	//底盘电机
	for(int i = 0; i<4; i++)
	{
		if(moto_chassis[i].heartbeat > 0)
		{
			moto_chassis[i].state = 1;
		}
		else
		{
			moto_chassis[i].state = 0;
		}
		moto_chassis[i].heartbeat = 0;
	}
	//刷子电机
	for(int i = 0; i<4; i++)
	{
		if(moto_brush[i].heartbeat > 0)
		{
			moto_brush[i].state = 1;
		}
		else
		{
			moto_brush[i].state = 0;
		}
		moto_brush[i].heartbeat = 0;
	}
	if(host_heartbeat > 0)
	{
		host_state = 1;
	}
	else
	{
		host_state = 0;
	}
	host_heartbeat = 0;
    osDelay(500);
  }
  /* USER CODE END StartMtHrtBt */
}

/* USER CODE BEGIN Header_StartFanCtrl */
/**
* @brief Function implementing the myFanCtrl thread.
* @param argument: Not used
* @retval None
*/
uint32_t fan_spd = 0;
/* USER CODE END Header_StartFanCtrl */
void StartFanCtrl(void *argument)
{
  /* USER CODE BEGIN StartFanCtrl */
  /* Infinite loop */
  for(;;)
  {
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, fan_spd);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, fan_spd);
    osDelay(10);
  }
  /* USER CODE END StartFanCtrl */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
