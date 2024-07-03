/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef pTXHeader;
CAN_TxHeaderTypeDef pTXHeader2;
CAN_TxHeaderTypeDef pTXHeader3;
CAN_TxHeaderTypeDef pTXHeader4;
CAN_TxHeaderTypeDef pTXHeader5;
CAN_TxHeaderTypeDef pTXHeader6;
CAN_TxHeaderTypeDef pTXHeader7;
CAN_TxHeaderTypeDef pTXHeader8;
CAN_TxHeaderTypeDef pTXHeader9;
CAN_TxHeaderTypeDef pTXHeaderBV;
CAN_RxHeaderTypeDef pRXHeader;
CAN_FilterTypeDef sfilterconfig;
uint32_t pTxMailbox;
uint8_t count;
uint16_t rcount,rcount2,rcount3;
uint8_t adc_value[1]={0};
uint8_t deger=0;
int durum;
int durum2;



uint16_t adcValue = 0;
uint8_t adcValueHigh = 0;
uint8_t adcValueLow = 0;
uint8_t data[8]; // CAN mesajı veri alanı
uint8_t data2[8];
uint8_t data23[8];
uint8_t led_set=1;
uint8_t led_reset=0;
char rx_buffer[50];
 char tx_buffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_PWM_Duty(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR1 = pulse_length; //değer setlendi
}
void Set_PWM_Duty2(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR2 = pulse_length; //değer setlendi
}
void Set_PWM_Duty3(uint16_t duty) {
    // 0-255 arası gelen duty değerini 0-999 arası bir değere dönüştür
    uint16_t pulse_length = ((uint32_t)duty * 1000) / 4095; //1khz lik ve 12 bit adc olduğu için pwm değerini ayarlıyor
    TIM1->CCR3 = pulse_length; //değer setlendi
}
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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */





	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

  //Can baslatildi
  HAL_CAN_Start(&hcan);
  //interrupt için aktif edilme
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //mesaj geldi mesaj bekleniyor


  pTXHeader.DLC=1;  //1byte lık değer geldiğini ifade eder
  pTXHeader.IDE=CAN_ID_STD; //standart ID kullanılıcağı belirtir
  pTXHeader.RTR=CAN_RTR_DATA; //data gönderilmesini transmit yapıldığını belirtir
  pTXHeader.StdId=0x0156; //mesage ID numarasıdır

  pTXHeader2.DLC=1;
  pTXHeader2.IDE=CAN_ID_STD;
  pTXHeader2.RTR=CAN_RTR_DATA;
  pTXHeader2.StdId=0x0158;

  pTXHeader3.DLC=1;
  pTXHeader3.IDE=CAN_ID_STD;
  pTXHeader3.RTR=CAN_RTR_DATA;
  pTXHeader3.StdId=0x0160;

  pTXHeader4.DLC=1;
  pTXHeader4.IDE=CAN_ID_STD;
  pTXHeader4.RTR=CAN_RTR_DATA;
  pTXHeader4.StdId=0x0162;

  pTXHeader5.DLC=1;
  pTXHeader5.IDE=CAN_ID_STD;
  pTXHeader5.RTR=CAN_RTR_DATA;
  pTXHeader5.StdId=0x0163;

  pTXHeader6.DLC=1;
  pTXHeader6.IDE=CAN_ID_STD;
  pTXHeader6.RTR=CAN_RTR_DATA;
  pTXHeader6.StdId=0x0164;

  pTXHeader7.DLC=1;
  pTXHeader7.IDE=CAN_ID_STD;
  pTXHeader7.RTR=CAN_RTR_DATA;
  pTXHeader7.StdId=0x0165;

  pTXHeader8.DLC=1;
  pTXHeader8.IDE=CAN_ID_STD;
  pTXHeader8.RTR=CAN_RTR_DATA;
  pTXHeader8.StdId=0x0166;

  pTXHeader9.DLC=1;
  pTXHeader9.IDE=CAN_ID_STD;
  pTXHeader9.RTR=CAN_RTR_DATA;
  pTXHeader9.StdId=0x0167;



  sfilterconfig.FilterActivation=ENABLE;
  sfilterconfig.FilterBank=0;
  sfilterconfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  sfilterconfig.FilterIdHigh=0x0000;
  sfilterconfig.FilterIdLow=0x0000;
  sfilterconfig.FilterMaskIdHigh=0x0000;
  sfilterconfig.FilterMaskIdLow=0x0000;
  sfilterconfig.FilterMode=CAN_FILTERMODE_IDMASK;
  sfilterconfig.FilterScale=CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan, &sfilterconfig);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //HAL_ADC_Start(&hadc1);
 //deger =(uint8_t)(HAL_ADC_GetValue(&hadc1));
	  // ADC değerini okuyun

	 //adcValue = HAL_ADC_GetValue(&hadc1);

	  // 12 bitlik ADC değerini iki 8 bitlik parçaya bölün
	  //adcValueHigh = (adcValue >> 4) & 0xFF; // Yüksek 8 bit 12 bitlik veriyi 8 ve 4 bit paketliyor
	  //adcValueLow = (adcValue & 0x0F) << 4;  // Düşük 4 bit

	  //data[0] = adcValueHigh; //gönderilmesi için değerleri dizide tutuyor
	  //data[1] = adcValueLow;

//      HAL_CAN_AddTxMessage(&hcan, &pTXHeader, data, &pTxMailbox); //bu iki değeri gönderiyor
	//  HAL_CAN_AddTxMessage(&hcan, &pTXHeader2, data, &pTxMailbox);
	  //HAL_CAN_AddTxMessage(&hcan, &pTXHeader3, data, &pTxMailbox);
	 // adcValueHigh = data[0]; //okunan data değerleri 8 ve 4 bit olarak toplam 12 bit oluyor
	  //adcValueLow = data[1];

	 // 12 bitlik ADC değerini yeniden oluşturun
	 //rcount = (adcValueHigh << 4) | (adcValueLow >> 4); // değerleri toplayarak 16 bitlik değeşkene 12 bitlik adc değeri olcak şekilde atıyor

	  ////////////////uart////////////////////////////////////////////////////////////////////////

	  HAL_UART_Receive(&huart2, (uint8_t*)rx_buffer, 50, 100);

	  	  if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  			  rx_buffer[3]=='1' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='n'){

	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
	  		HAL_CAN_AddTxMessage(&hcan, &pTXHeader4, &led_set, &pTxMailbox);
	  		  //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"birinci led yandi."), 100);
	  	  }


	  	   else if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  				  rx_buffer[3]=='1' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='f' &&
	  				  rx_buffer[7]=='f'){
		  		HAL_CAN_AddTxMessage(&hcan, &pTXHeader5,&led_reset, &pTxMailbox);

	  		   //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"birinci led söndü."), 100);
	  	   }


	  	  if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  	 			  rx_buffer[3]=='2' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='n'){
	  		HAL_CAN_AddTxMessage(&hcan, &pTXHeader6, &led_set, &pTxMailbox);
	  		  //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"ikinci led yandi."), 100);
	  	  }


	  	 	   else if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  	 				  rx_buffer[3]=='2' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='f' &&
	  	 				  rx_buffer[7]=='f'){
	  	 		HAL_CAN_AddTxMessage(&hcan, &pTXHeader7, &led_reset, &pTxMailbox);
	  	 		 // HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"ikinci led söndü."), 100);
	  	 	   }


	  	  if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  	 			  rx_buffer[3]=='3' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='n'){
	  		HAL_CAN_AddTxMessage(&hcan, &pTXHeader8, &led_set, &pTxMailbox);
	  		 // HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"ücüncü led yandi."), 100);
	  	  }


	  	 	   else if(rx_buffer[0]=='l' && rx_buffer[1]=='e' && rx_buffer[2]=='d' &&
	  	 				  rx_buffer[3]=='3' && rx_buffer[4]==' ' && rx_buffer[5]=='o' && rx_buffer[6]=='f' &&
	  	 				  rx_buffer[7]=='f'){
	  	 		HAL_CAN_AddTxMessage(&hcan, &pTXHeader9, &led_reset, &pTxMailbox);
	  	 		 // HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sprintf(tx_buffer,"ücüncü led söndü."), 100);
	  	 	   }



	  	  ////////////////////////////uart_end///////////////////////////////////////////////////


     Set_PWM_Duty(rcount); //pwm fonksiyonuna bu değeri gönderiyor
     Set_PWM_Duty2(rcount2);
     Set_PWM_Duty3(rcount3);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
