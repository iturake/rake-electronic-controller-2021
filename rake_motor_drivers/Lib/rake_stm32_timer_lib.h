#ifndef RAKE_STM32_TIMER_LIB_H
#define RAKE_STM32_TIMER_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Private structures ----------------------------------------------------------*/

struct timerStruct {
	uint16_t velocityCalculator_u16;
	uint16_t slowStartMotor_u16;
	uint16_t communicationUART_u16;
	uint16_t communicationCANBUS_u16;
	uint16_t ledDriver_u16;
	uint16_t pidCalculator_u16;
};


/* Private function prototypes -----------------------------------------------*/

void RAKE_TIM2_Init(void);
void RAKE_TIM3_Init(void);
void RAKE_TIM4_Init(void);
void RAKE_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void RAKE_Error_Handler(void);




#ifdef _cplusplus
  }
#endif

#endif
