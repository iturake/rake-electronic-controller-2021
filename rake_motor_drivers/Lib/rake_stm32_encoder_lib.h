#ifndef RAKE_STM32_ENCODER_LIB_H
#define RAKE_STM32_ENCODER_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private structures ----------------------------------------------------------*/

struct encoderStruct {
	uint32_t counter_u32;
	_Bool measuredDirection_bool;
	float measuredSpeed_f32;
};


/* Private functions -----------------------------------------------*/




#ifdef _cplusplus
  }
#endif

#endif
