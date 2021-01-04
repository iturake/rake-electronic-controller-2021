
/* Private includes ----------------------------------------------------------*/
#include "rake_stm32_encoder_lib.h"


/* Extern structures -----------------------------------------------*/

extern ENCODER_HandleTypeDef rencoder1;

/* Private functions -----------------------------------------------*/

void RAKE_ENCODER_Init(void) {
	rencoder1.counter_u32 = 0;
	rencoder1.measuredDirection_bool = 0;
	rencoder1.measuredSpeed_f32 = 0.0;
}


void RAKE_Measure_Speed(TIMER_HandleTypeDef *timer, FLAG_HandleTypeDef *flag, ENCODER_HandleTypeDef *encoder){
	if(TIM2->CR1 == 1) {
		encoder->measuredDirection_bool = 0;
		encoder->counter_u32 = (uint32_t)(TIM2->CNT);
		if(timer->velocityCalculator_u16 > VELOCITY_TIME) {
			TIM2->CNT = 0;
			encoder->measuredSpeed_f32 = ((float)encoder->counter_u32 / ENCODER_PULS) * 60.0;
			timer->velocityCalculator_u16 = 0;
			flag->LED.motorForward_bit = 1;
			flag->LED.motorBackward_bit = 0;
		}
	} else if(TIM2->CR1 == 17) {
		encoder->measuredDirection_bool = 1;
		encoder->counter_u32 = 65535 - (uint32_t)TIM2->CNT;
		if(timer->velocityCalculator_u16 > VELOCITY_TIME) {
			TIM2->CNT = 65535;
			encoder->measuredSpeed_f32 = ((float)encoder->counter_u32 / ENCODER_PULS) * 60.0;
			timer->velocityCalculator_u16 = 0;
			flag->LED.motorForward_bit = 0;
			flag->LED.motorBackward_bit = 1;
		}
	}
}
