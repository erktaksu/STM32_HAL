/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
/* USER CODE BEGIN EV */
extern uint8_t count;
extern uint16_t rcount,rcount2,rcount3;
extern CAN_TxHeaderTypeDef pTXHeader;
extern CAN_RxHeaderTypeDef pRXHeader;
extern uint32_t pTxMailbox;
extern int durum;
extern uint8_t data[8];
extern uint8_t data2[8];
extern uint8_t data23[8];
extern uint16_t adcValue ;
extern uint8_t adcValueHigh ;
extern uint8_t adcValueLow ;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
    for(int i=0; i<1000; i++);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
  {
	  count++;
	  HAL_CAN_AddTxMessage(&hcan, &pTXHeader, &count, &pTxMailbox);
  }
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

if((HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRXHeader, data))==HAL_OK)
{

	if (pRXHeader.StdId == 0x0156 && pRXHeader.DLC == 1) { //mesage ID sini kontrol eder
	            // Mesaj 1 alındı
	            adcValueHigh = data[0];
	            adcValueLow = data[1];
	            rcount = (adcValueHigh << 4) | (adcValueLow >> 4);


	}
	if (pRXHeader.StdId == 0x0158 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
	            adcValueHigh = data[0];
	            adcValueLow = data[1];
	            rcount2 = (adcValueHigh << 4) | (adcValueLow >> 4);


	}


	if (pRXHeader.StdId == 0x0160 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
	            adcValueHigh = data[0];
	            adcValueLow = data[1];
	            rcount3 = (adcValueHigh << 4) | (adcValueLow >> 4);


	}

	if (pRXHeader.StdId == 0x0162 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);

	}

	if (pRXHeader.StdId == 0x0163 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);

	}
	if (pRXHeader.StdId == 0x0164 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);

	}

	if (pRXHeader.StdId == 0x0165 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);

	}
	if (pRXHeader.StdId == 0x0166 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);

	}

	if (pRXHeader.StdId == 0x0167 && pRXHeader.DLC == 1) {
	            // Mesaj 1 alındı
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);

	}

if(rcount%2==0)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
}
else
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
}


}
else if((HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRXHeader, data))==HAL_ERROR)
{
	durum = 1;

}
else if((HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRXHeader, data))==HAL_BUSY)
{
	durum = 2;

}
else if((HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRXHeader, data))==HAL_TIMEOUT)
{
	durum = 3;

}
else
{
	durum = 10;

}

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
