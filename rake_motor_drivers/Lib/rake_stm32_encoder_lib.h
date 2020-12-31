#ifndef RAKE_STM32_ENCODER_LIB_H
#define RAKE_STM32_ENCODER_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "rake_stm32_timer_lib.h"
#include "rake_stm32_uart_lib.h"
#include "rake_stm32_extra_lib.h"

/* Private structures ----------------------------------------------------------*/

typedef struct encoderStruct {
	uint32_t counter_u32;
	_Bool measuredDirection_bool;
	float measuredSpeed_f32;
} ENCODER_HandleTypeDef;

/* Private functions -----------------------------------------------*/

void RAKE_Measure_Speed(TIMER_HandleTypeDef *timer, FLAG_HandleTypeDef *flag, ENCODER_HandleTypeDef *encoder);


#ifdef _cplusplus
  }
#endif

#endif
