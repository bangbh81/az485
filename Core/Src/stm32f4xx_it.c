/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include "s2cCore.h"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __HAL_DMA_SET_MAR(__HANDLE__, __ADDRESS__) ((__HANDLE__)->Instance->M0AR = (uint32_t)(__ADDRESS__))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern osMessageQueueId_t sQrecv, sQsend, eQrecv, eQsend;
extern uint8_t u8S2EBuffer[QUEUE_BUFFER_RX_SIZE];
extern uint8_t u8E2SBuffer[QUEUE_BUFFER_TX_SIZE];
extern uint32_t u32S2EBufLoc;
extern uint32_t u32E2SBufLoc;
extern uint32_t u32S2ECurrDMASize;
extern uint32_t u32E2SCurrDMASize;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	//this for the UART2 RX DMA interrupt
	osStatus_t osStatus;
	mQueue qItem;
	HAL_TIM_Base_Stop_IT(&htim2);
	qItem.size = u32S2ECurrDMASize - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	qItem.start = u8S2EBuffer + u32S2EBufLoc;
	qItem.isSplit = 1;
	osStatus = osMessageQueuePut(sQrecv, &qItem, NULL, NULL);
	u32S2EBufLoc += qItem.size;
	if(u32S2EBufLoc == QUEUE_BUFFER_RX_SIZE)
		u32S2EBufLoc = 0;

	HAL_UART_DMAStop(&huart2);
//	huart2.RxState = HAL_UART_STATE_READY;
//	hdma_usart2_rx.State = HAL_DMA_STATE_READY;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);//Peter Enable "Receive data register not empty" interrupt
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);//clear flag

	if(u32S2EBufLoc + E_BUFFER_SIZE > QUEUE_BUFFER_RX_SIZE)
	{
		u32S2ECurrDMASize = QUEUE_BUFFER_RX_SIZE - u32S2EBufLoc;//
		HAL_UART_Receive_DMA(&huart2, u8S2EBuffer + u32S2EBufLoc, u32S2ECurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx, u8S2EBuffer + u32S2EBufLoc + qItem.size);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	else
	{
		u32S2ECurrDMASize = E_BUFFER_SIZE;
		HAL_UART_Receive_DMA(&huart2, u8S2EBuffer + u32S2EBufLoc, u32S2ECurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */
	HAL_UART_DMAResume(&huart2);
	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TCIF1_5);
  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
	//HAL_UART_DMAStop(&huart2);
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */
	huart2.gState = HAL_UART_STATE_READY;
	__HAL_UNLOCK(&hdma_usart2_tx);
	hdma_usart2_tx.State = HAL_DMA_STATE_READY;
	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF2_6|DMA_FLAG_HTIF2_6);
  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	osStatus_t osStatus;
	mQueue qItem;
	HAL_TIM_Base_Stop_IT(&htim2);
	qItem.size = u32S2ECurrDMASize - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	qItem.start = u8S2EBuffer + u32S2EBufLoc;
	qItem.isSplit = 0;
	osStatus = osMessageQueuePut(sQrecv, &qItem, NULL, NULL);//Need to be checked.
	u32S2EBufLoc += qItem.size;

	HAL_UART_DMAStop(&huart2);
//	huart2.RxState = HAL_UART_STATE_READY;
//	hdma_usart2_rx.State = HAL_DMA_STATE_READY;
//	__HAL_UNLOCK(&hdma_usart2_rx);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);//Peter Enable "Receive data register not empty" interrupt
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);//clear flag

	if(u32S2EBufLoc + E_BUFFER_SIZE > QUEUE_BUFFER_RX_SIZE)
	{
		u32S2ECurrDMASize = QUEUE_BUFFER_RX_SIZE - u32S2EBufLoc;
		HAL_UART_Receive_DMA(&huart2, u8S2EBuffer + u32S2EBufLoc, u32S2ECurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	else
	{
		u32S2ECurrDMASize = E_BUFFER_SIZE;
		HAL_UART_Receive_DMA(&huart2, u8S2EBuffer + u32S2EBufLoc, u32S2ECurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	HAL_UART_DMAResume(&huart2);
//	HAL_DMA_Abort(&hdma_usart2_rx);
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */
	__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
	//osKernelUnlock();
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	osStatus_t osStatus;
	mQueue qItem;

	HAL_TIM_Base_Stop_IT(&htim3);

	qItem.size = u32E2SCurrDMASize - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
	qItem.start = u8E2SBuffer + u32E2SBufLoc;
	qItem.isSplit = 0;
	osStatus = osMessageQueuePut(eQrecv, &qItem, NULL, NULL);//Need to be checked.
	u32E2SBufLoc += qItem.size;

	HAL_UART_DMAStop(&huart6);
//	huart6.RxState = HAL_UART_STATE_READY;
//	hdma_usart6_rx.State = HAL_DMA_STATE_READY;
//	__HAL_UNLOCK(&hdma_usart6_rx);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);//Peter Enable "Receive data register not empty" interrupt
	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);//clear flag

	if(u32E2SBufLoc + E_BUFFER_SIZE > QUEUE_BUFFER_TX_SIZE)//if
	{
		u32E2SCurrDMASize = QUEUE_BUFFER_TX_SIZE - u32E2SBufLoc;
		HAL_UART_Receive_DMA(&huart6, u8E2SBuffer + u32E2SBufLoc, u32E2SCurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	else
	{
		u32E2SCurrDMASize = E_BUFFER_SIZE;
		HAL_UART_Receive_DMA(&huart6, u8E2SBuffer + u32E2SBufLoc, u32E2SCurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	HAL_UART_DMAResume(&huart6);


  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	//osKernelLock();
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */
	__HAL_TIM_SET_COUNTER(&htim2,0);	// set counter 0
	HAL_TIM_Base_Start_IT(&htim2);	// start timer

  __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_RXNE|UART_FLAG_ORE);
  //osKernelUnlock();
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	osStatus_t osStatus;
	mQueue qItem;
	HAL_TIM_Base_Stop_IT(&htim3);
	qItem.size = u32E2SCurrDMASize - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
	qItem.start = u8E2SBuffer + u32E2SBufLoc;
	qItem.isSplit = 1;
	osStatus = osMessageQueuePut(eQrecv, &qItem, NULL, NULL);//Need to be checked.
	u32E2SBufLoc += qItem.size;
	if(u32E2SBufLoc == QUEUE_BUFFER_TX_SIZE)
		u32E2SBufLoc = 0;

	HAL_UART_DMAStop(&huart6);
//	huart6.RxState = HAL_UART_STATE_READY;
//
//	hdma_usart6_rx.State = HAL_DMA_STATE_READY;
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);//Peter Enable "Receive data register not empty" interrupt
	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);//clear flag
//	__HAL_UNLOCK(&hdma_usart6_rx);

	if(u32E2SBufLoc + E_BUFFER_SIZE > QUEUE_BUFFER_TX_SIZE)
	{
		u32E2SCurrDMASize = QUEUE_BUFFER_TX_SIZE - u32E2SBufLoc;
		HAL_UART_Receive_DMA(&huart6, u8E2SBuffer + u32E2SBufLoc, u32E2SCurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	else
	{
		u32E2SCurrDMASize = E_BUFFER_SIZE;
		HAL_UART_Receive_DMA(&huart6, u8E2SBuffer + u32E2SBufLoc, u32E2SCurrDMASize);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);//disable HT interrupt
//		__HAL_DMA_SET_MAR(&hdma_usart2_rx,u8S2EBuffer + u32S2EBufLoc);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,u32S2ECurrDMASize);
	}
	HAL_UART_DMAResume(&huart6);
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5|DMA_FLAG_HTIF1_5);
  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */
	//HAL_UART_DMAStop(&huart6);
  /* USER CODE END DMA2_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */
	//HAL_UART_DMAResume(&huart6);
	huart6.gState = HAL_UART_STATE_READY;

	__HAL_UNLOCK(&hdma_usart6_tx);
	hdma_usart6_tx.State = HAL_DMA_STATE_READY;
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_FLAG_TCIF2_6|DMA_FLAG_HTIF2_6);
  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  /* USER CODE BEGIN USART6_IRQn 1 */
	__HAL_TIM_SET_COUNTER(&htim3,0);	// set counter 0
	HAL_TIM_Base_Start_IT(&htim3);	// start timer

	__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_RXNE|UART_FLAG_ORE);
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
