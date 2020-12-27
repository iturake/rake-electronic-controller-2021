
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_extra_lib.h"

/* Private structures -----------------------------------------------*/



/* Private variables -----------------------------------------------*/

uint32_t motorBackward_Pin = TIM_CHANNEL_1;
uint32_t motorForward_Pin = TIM_CHANNEL_2;
uint32_t encoderA_Pin = TIM_CHANNEL_1;
uint32_t encoderB_Pin = TIM_CHANNEL_2;


/* Private functions -----------------------------------------------*/

float RAKE_Convert(uint8_t convertMode_u8, float convertingValue_f32) {
	
	uint32_t convert_result ;
	
	switch(convertMode_u8){
		case VOLTAGE_TO_PWM:
			convert_result = PWM_ZERO + (convertingValue_f32 / VOLTAGE_MAX) * (PWM_MAX - PWM_ZERO);
			break;
		case PWM_TO_VOLTAGE:
			convert_result = (convertingValue_f32 / PWM_MAX) * VOLTAGE_MAX;
			break;
		case RPM_TO_PWM:
			convert_result = (convertingValue_f32 / RPM_MAX) * PWM_MAX;
			break;
		case PWM_TO_RPM:
			convert_result = (convertingValue_f32 / PWM_MAX) * RPM_MAX;
			break;
	}
	return convert_result;
}

void RAKE_Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

