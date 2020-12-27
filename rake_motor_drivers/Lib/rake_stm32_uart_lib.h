#ifndef RAKE_STM32_UART_LIB_H
#define RAKE_STM32_UART_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"


/* Private structures ----------------------------------------------------------*/

struct uartStruct {
	unsigned char rxData[8];
	unsigned char txBuffer[30];
	unsigned char rxBuffer[30];
	uint16_t txBufferLen;
};

struct flagStruct {
	struct {
		_Bool motorForward_bit;
		_Bool motorBackward_bit;
		_Bool UART_bit;
		_Bool CANBUS_bit;
		_Bool adminMode_bit;
		_Bool testMode_bit;
		_Bool normalMode_bit;
		_Bool ERROR_bit;
	} LED;
	struct {
		uint8_t rxIndex_bool;
		_Bool rxComplete_bool;
	} UART;
};




/* Private functions -----------------------------------------------*/

void RAKE_USART1_UART_Init(void);









#ifdef _cplusplus
  }
#endif

#endif
