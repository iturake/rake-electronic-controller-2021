#ifndef RAKE_STM32_UART_LIB_H
#define RAKE_STM32_UART_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Private variables ----------------------------------------------------------*/





/* Private structures ----------------------------------------------------------*/

typedef struct uartStruct {
	unsigned char rxData[8];
	unsigned char txBuffer[30];
	unsigned char rxBuffer[30];
	uint16_t txBufferLen;
}RAKE_UART_HandleTypeDef;



/* Private functions -----------------------------------------------*/

void RAKE_USART1_UART_Init(void);











#ifdef _cplusplus
  }
#endif

#endif
