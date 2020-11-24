/*
 * s2cCore.h
 *
 *  Created on: Oct 13, 2020
 *      Author: peter
 */

#ifndef INC_S2CCORE_H_
#define INC_S2CCORE_H_

#include "stm32f4xx_hal.h"

#define QUEUE_BUFFER_TX_SIZE	1024*4	//4k bytes
#define QUEUE_BUFFER_RX_SIZE	1024*4	//4k bytes
#define	QUEUE_MAX_SIZE			20		//20
#define S2E_DMA_SIZE			1024
#define E2S_DMA_SIZE			1024

#define E_BUFFER_SIZE			1024

#define DELIMIT_TIME_S			1000
#define DELIMIT_TIME_E			1000

#define CMD_NAME	"{\"OutMsg\":\""
#define CMD_SIZE	11

//#define WIZFI360_PUB_FRONT	"AT+WINC_PUB_EVENT={\"InMsg\":\""	//"AT+WINC_PUB_EVENT={\"InMsg\":\""
#define WIZFI360_PUB_FRONT	"AT+WINC_PUB_EVENT={\"InMsg\":\""	//"AT+WINC_PUB_EVENT={"EventDataFormat":{\"InMsg\":\""
#define WIZFI360_PUB_END		"\"}\r\n"						//"\"}}\r\n"

typedef enum {
	MSGTYPE_NULL = 0x00,
	MSGTYPE_CMD = 0x01,
	MSGTYPE_BYPASS = 0x02
}msgType;

typedef struct _MQUEUE
{
	uint32_t size;//Size of Buffer
	uint8_t * start;//Buffer
	uint8_t isSplit;
	//msgType type;
}mQueue;

void tskS2CCore( void *argument );
void tsksQSendTrigger(void *argument);
void tskeQSendTrigger(void *argument);

void tskLED(void *argument);


#endif /* INC_S2CCORE_H_ */
