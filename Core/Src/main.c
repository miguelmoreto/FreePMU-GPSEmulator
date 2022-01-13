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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

uint8_t flagTIM2 = 0;
char time[30];
char date[30];
char gps_string[100];
int tempcounter = 0;
int pps_counter = 0;
RTC_TimeTypeDef sTime;
RTC_TimeTypeDef rxTime;
RTC_DateTypeDef sDate;
RTC_DateTypeDef rxDate;
uint8_t UART6_rxCommand[1] = {0};
uint8_t UART6_rxBuffer[17] = {0}; // Buffer where DMA will store the received date and time from serial.
uint8_t UART6_rxFlag = 0;
uint8_t rxDateTimeFlag = 0;

// Serial terminal messages:
const char welcome_msg[] = "\r\nFreePMU GPS emulator version 1.\r\n";
const char reset_dt_msg[] = "\r\nDate and time reset to 2022-01-01 12:00\r\n";
const char help_msg[] = "\r\nThis is the help message.\r\n\r\n\t h: Show this help message.\r\n\t p: Print the last send NMEA message.\r\n\t P: Print formatted date and time.\r\n\t s: Set the time and date.\r\n";
const char prompt1_msg[] = "\r\nEnter the date and time string (enable echo in your terminal).\r\nFomart: hh,mm,ss,DD,MM,YY\r\n=> ";
const char prompt2_msg[] = "\r\nString received: ";
const char prompt3_msg[] = "\r\nDate and time set to:\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
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
  uint8_t a;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Transmit(&huart6, (uint8_t*)welcome_msg,sizeof(welcome_msg),1000);
  HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, sizeof(UART6_rxCommand));

  // Check user button press. If pressed, RTC clock is reseted to the subsequent
  // date and time.
  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
	  // Specified values are in BCD format.
	  sTime.Hours = 0x12;
	  sTime.Minutes = 0x00;
	  sTime.Seconds = 0x00;
	  sTime.SecondFraction = 0x00;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0x01;
	  sDate.Year = 0x22;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_UART_Transmit(&huart6, (uint8_t*)reset_dt_msg, sizeof(reset_dt_msg), 1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (pps_counter > 29){
		  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		  //"GPRMC,190430,A,4812.3038,S,07330.7690,W,3.7,3.8,090210,13.7,E,D*26\n";
		  sprintf(date,"Date: %02d.%02d.%02d\t",sDate.Date,sDate.Month,sDate.Year);
		  sprintf(time,"Time: %02d.%02d.%02d\r\n",sTime.Hours,sTime.Minutes,sTime.Seconds);
		  sprintf(gps_string,"$GPRMC,%02d%02d%02d,A,2730.00,S,048300.00,W,0.0,0.0,%02d%02d%02d,20.3,E*26\r\n",sTime.Hours,sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year);

		  HAL_UART_Transmit(&huart1, (uint8_t*)gps_string, sizeof(gps_string),1000);
		  tempcounter++;
		  pps_counter = 0;
		  flagTIM2 = 0;
	  }

	  if(UART6_rxFlag){
		  if (*UART6_rxCommand == 'h'){
			  HAL_UART_Transmit(&huart6, (uint8_t*)help_msg, sizeof(help_msg), 1000);
			  HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, sizeof(UART6_rxCommand)); // Enable reception of a command
		  }
		  else if(*UART6_rxCommand == 'p'){
			  HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2,1000);
			  HAL_UART_Transmit(&huart6, (uint8_t*)gps_string, sizeof(gps_string),1000);
			  HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, sizeof(UART6_rxCommand)); // Enable reception of a command
		  }
		  else if(*UART6_rxCommand == 'P'){
			  HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2,1000);
			  HAL_UART_Transmit(&huart6, (uint8_t*)date, sizeof(date),1000);
			  HAL_UART_Transmit(&huart6, (uint8_t*)time, sizeof(time),1000);
			  HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, sizeof(UART6_rxCommand)); // Enable reception of a command
		  }
		  else if(*UART6_rxCommand == 's'){
			  if (rxDateTimeFlag){ // Decode buffer
				  //HAL_UART_Transmit(&huart6, (uint8_t*)prompt2_msg, sizeof(prompt2_msg), 1000);
				  sscanf((char *)UART6_rxBuffer,"%02d,%02d,%02d,%02d,%02d,%02d",(int *)&rxTime.Hours,(int *)&rxTime.Minutes,(int *)&rxTime.Seconds,(int *)&a,(int *)&rxDate.Month,(int *)&rxDate.Year);
				  rxDate.Date = a; // Due to sscanf bizarre behavior.
				  sprintf(date,"\r\nDate: %02d.%02d.%02d\t",rxDate.Date,rxDate.Month,rxDate.Year);
				  sprintf(time,"Time: %02d.%02d.%02d\r\n",rxTime.Hours,rxTime.Minutes,rxTime.Seconds);
				  HAL_UART_Transmit(&huart6, (uint8_t*)prompt3_msg, sizeof(prompt3_msg), 1000);
				  HAL_UART_Transmit(&huart6, (uint8_t*)date, sizeof(date),1000);
				  HAL_UART_Transmit(&huart6, (uint8_t*)time, sizeof(time),1000);
				  rxDateTimeFlag = 0;
				  HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, sizeof(UART6_rxCommand)); // Enable reception of a command
			  }else{ // Prompt for setting:
				  HAL_UART_Transmit(&huart6, (uint8_t*)prompt1_msg, sizeof(prompt1_msg), 1000);
				  HAL_UART_Receive_DMA(&huart6, UART6_rxBuffer, sizeof(UART6_rxBuffer)); // Enable reception of the buffer.
				  rxDateTimeFlag = 1; // flag for the next time the interrupt is triggered  (buffer full).
			  }
		  }
		  // Reset flag:
		  UART6_rxFlag = 0;

	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */


  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2799999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 279999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback of Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim2 ){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		//flagTIM2 = 1;
		pps_counter++;
	}

}

// Callback of the UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart6){
		UART6_rxFlag = 1;
		//HAL_UART_Receive_DMA(&huart6, UART6_rxCommand, 2);
	}

}

/* USER CODE END 4 */

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
