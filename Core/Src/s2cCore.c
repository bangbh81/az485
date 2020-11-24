/*
 * S2CCore.c
 *
 *  Created on: Oct 13, 2020
 *      Author: peter
 */

#include "s2cCore.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "core_json.h"

#define __DEBUG_S2ECORE__

extern osMessageQueueId_t sQrecv, sQsend, eQrecv, eQsend;
extern uint8_t u8S2EBuffer[QUEUE_BUFFER_RX_SIZE];
extern uint8_t u8E2SBuffer[QUEUE_BUFFER_TX_SIZE];
extern uint32_t u32S2EBufLoc;
extern uint32_t u32E2SBufLoc;
extern uint32_t u32S2ECurrDMASize;
extern uint32_t u32E2SCurrDMASize;

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim5;


osThreadId_t tsksQSendTriggerHandle;
const osThreadAttr_t tsksQSendTrigger_attributes = {
  .name = "sQsendDMATrigger",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t tskeQSendTriggerHandle;
const osThreadAttr_t tskeQSendTrigger_attributes = {
  .name = "eQsendDMATrigger",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t tskLEDHandle;
const osThreadAttr_t tskLED_attributes = {
  .name = "LED",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

static char cmdName[] = CMD_NAME;
static char wizfi360_pub_front[] = WIZFI360_PUB_FRONT;
static char wizfi360_pub_end[] = WIZFI360_PUB_END;

static mQueue pub_front = {
		sizeof(wizfi360_pub_front)-1,
		wizfi360_pub_front,
		0,
		0
};

static mQueue pub_end = {
		sizeof(wizfi360_pub_end)-1,
		wizfi360_pub_end,
		0,
		0
};

void tskS2CCore(void *argument)
{
	mQueue S2EQ,E2SQ;
	uint8_t *pMsg, pTmpMsg;
	uint32_t tmpSize;
	HAL_GPIO_TogglePin(GPIOB, WizFi_Reset_Pin);//reset GPIO
	tsksQSendTriggerHandle = osThreadNew(tsksQSendTrigger, NULL, &tsksQSendTrigger_attributes);//Enable S2E Queue send
	tskeQSendTriggerHandle = osThreadNew(tskeQSendTrigger, NULL, &tskeQSendTrigger_attributes);//Enable E2S Queue send
	tskLEDHandle = osThreadNew(tskLED, NULL, &tskLED_attributes);//Enable E2S Queue send

	while(1)
	{
		//Customization area
		//BYPASS
//		if(osMessageQueueGet(sQrecv, &S2EQ, NULL, NULL) == osOK){
//			osMessageQueuePut(eQsend, &S2EQ, NULL, NULL);//bypass
//		}
//		if(osMessageQueueGet(eQrecv, &E2SQ, NULL, NULL) == osOK){
//			osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);//bypass
//		}

		/*
		 * WIZFI360
		 */
		if(osMessageQueueGet(eQrecv, &E2SQ, NULL, NULL) == osOK){//check if the message is in queue from wizfi360
			//1. copy the memory
			pMsg = pvPortMalloc(E2SQ.size);// memory allocation
			if(pMsg == NULL){//if fail to allocate memory
				printf("Error!!!\r\n");
				break;
			}
			memcpy(pMsg, E2SQ.start, E2SQ.size); // message copy to the new memory
			E2SQ.start = pMsg;// Queue item update

			//2. check if the message is splited ? re allocate the memory : break;ready
			if(E2SQ.isSplit){
				pTmpMsg = pvPortMalloc(E2SQ.size);// memory allocation
				memcpy(pTmpMsg, pMsg, E2SQ.size);// copy memory
				vPortFree(pMsg);// free the previous memory
				tmpSize = E2SQ.size;
				while(osMessageQueueGet(sQrecv, &S2EQ, NULL, NULL) != osOK){}//It's dangerous, we need to add timer or something..
				pMsg = pvPortMalloc(E2SQ.size + tmpSize);// memory allocation again
				memcpy(pMsg, pTmpMsg, tmpSize);// first one
				memcpy(pMsg+tmpSize, E2SQ.start, E2SQ.size);// Second one
				free(pTmpMsg);// Free temp memory
				E2SQ.start = pMsg;
				E2SQ.size = E2SQ.size + tmpSize;
				E2SQ.isSplit = 0;
			}

			//Check type of packet
			uint8_t msgType[3];
			osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);//bypass
			uint8_t *cmdptr = strstr(E2SQ.start,cmdName);//check if the message is for the command
//			//parse the payload using command name
//			uint8_t mid[8];
//			uint8_t *cmdptr = strstr(E2SQ.start,cmdName);//check if the message is for the command
//			if(cmdptr){//if yes
//				E2SQ.start = cmdptr + CMD_SIZE;		//recalculate the start address of the payload
//				E2SQ.size = strlen(E2SQ.start) - 4;	//recalculate the size of the payload
//				osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);
//			} else {//if no
//				osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);//bypass
//			}
		}

		if(osMessageQueueGet(sQrecv, &S2EQ, NULL, NULL) == osOK){//Check if the message is in queue from serial
			//at command should be composed here
			//AT+WINC_PUB_EVENT={"EventDataFormat":{"InMsg":"Hello world!!"}}$0d$0a
			//way #1
			//buffer -> sprintf
			//osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);//let's bypass at first
			/*
			 * osMessageQueuePut(sQsend, &, NULL, NULL);// send *AT+WINC_PUB_EVENT={"InMsg":"*
			 * osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);// send *Hello world!!*
			 * osMessageQueuePut(sQsend, &E2SQ, NULL, NULL);// send *"}}$0d$0a*
			 */
			osMessageQueuePut(eQsend, &pub_front, NULL, NULL);// send *AT+WINC_PUB_EVENT={\"InMsg\":\"*
			osMessageQueuePut(eQsend, &S2EQ, NULL, NULL);// send *Hello world!!*
			if(S2EQ.isSplit){
				while(osMessageQueueGet(sQrecv, &S2EQ, NULL, NULL) != osOK){}
				osMessageQueuePut(eQsend, &S2EQ, NULL, NULL);
			}
			osMessageQueuePut(eQsend, &pub_end, NULL, NULL);// send *\"}}\r\n*
		}
	}
}

/*
 * UART2 DMA Transmit Trigger
 * DMA1 Stream 5: RX, DMA1 Stream 6 : TX
 */
void tsksQSendTrigger(void *argument)
{
	mQueue messageQueue;

	while(1)
	{
		if(osMessageQueueGet(sQsend, &messageQueue, NULL, NULL) == osOK){
			HAL_UART_Transmit_DMA(&huart2, messageQueue.start, messageQueue.size);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);//disable HT interrupt
#ifdef __DEBUG_S2ECORE__
			printf("sQsend DMA send:%d\r\n",messageQueue.size);
#endif
			while(hdma_usart2_tx.State != HAL_DMA_STATE_READY){}
			//Free here
			vPortFree(messageQueue.start);
		}
	}
}

/*
 * UART6 DMA Transmit Trigger
 * DMA2 Stream 1: RX, DMA2 Stream 6 : TX
 */
void tskeQSendTrigger(void *argument)
{
	mQueue messageQueue;

	while(1)
	{
		if(osMessageQueueGet(eQsend, &messageQueue, NULL, NULL) == osOK){//DMA2 Stream 6 : TX
			HAL_UART_Transmit_DMA(&huart6, messageQueue.start, messageQueue.size);
			__HAL_DMA_DISABLE_IT(&hdma_usart6_tx, DMA_IT_HT);//disable HT interrupt
#ifdef __DEBUG_S2ECORE__
			printf("eQsend DMA send:%d\r\n",messageQueue.size);
#endif
			while(hdma_usart6_tx.State != HAL_DMA_STATE_READY){}
			//Free here
			vPortFree(messageQueue.start);
		}
	}
}

void tskLED(void *argument){
	while(1){
		osDelay(100);
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
	}
}
