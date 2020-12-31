
/* Private includes ----------------------------------------------------------*/
#include "rake_stm32_encoder_lib.h" 

/* Private structures -----------------------------------------------*/

// struct encoderStruct encoder = {0,0,0.0};  --> Bu kaliplari kullanmaktan kacinalim

/* Private functions -----------------------------------------------*/

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
