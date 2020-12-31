#ifndef RAKE_STM32_MOTOR_LIB_H
#define RAKE_STM32_MOTOR_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"


/* Private structures ----------------------------------------------------------*/

typedef struct motorStruct {
	uint16_t pwmValue_u16;
	uint16_t pwmLastValue_u16;
	_Bool lastDirection_bool;
	struct {
		uint16_t PWM_u16;
		float RPM_f32;
		_Bool direction;
	} desired;
}MOTOR_HandleTypeDef;








/* Private functions -----------------------------------------------*/





#ifdef _cplusplus
  }
#endif

#endif
