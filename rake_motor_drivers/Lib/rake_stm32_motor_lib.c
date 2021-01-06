
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_motor_lib.h"

/* Extern variables -----------------------------------------------*/

extern uint32_t motorBackward_Pin;
extern uint32_t motorForward_Pin;
extern uint32_t encoderA_Pin; 
extern uint32_t encoderB_Pin; 

extern TIM_HandleTypeDef htim3;

extern MOTOR_HandleTypeDef rmotor1;


/* Private functions -----------------------------------------------*/

// Motor structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_MOTOR_Init(void) {
	rmotor1.pwmValue_u16 = 0;
	rmotor1.pwmLastValue_u16 = 0;
	rmotor1.lastDirection_bool = 1;
	rmotor1.desired.PWM_u16 = 0;
	rmotor1.desired.RPM_f32 = 0;
	rmotor1.desired.direction = 0;
}

// Motor s�rme fonksiyonu asagida tanimlandi .
void RAKE_Drive_Motor(float voltageValue, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor) {
	motor->pwmValue_u16 = (int)RAKE_Convert(VOLTAGE_TO_PWM, voltageValue);
	uint32_t motorPin;
	if(motor->desired.direction == 0) {
		//HAL_GPIO_WritePin(motorBackwardEnable_GPIO_Port, motorBackwardEnable_Pin, 1);
		//HAL_GPIO_WritePin(motorForwardEnable_GPIO_Port, motorForwardEnable_Pin, 0);
		if(motor->desired.direction != encoder->measuredDirection_bool) {
		for(int a = motor->pwmValue_u16 ; a > 0; a -= 3)  {
			__HAL_TIM_SET_COMPARE(&htim3, motorForward_Pin, a);
			HAL_Delay(1);
		}
	}
		motorPin = motorBackward_Pin;
	} else if (motor->desired.direction == 1) {
		//HAL_GPIO_WritePin(motorBackwardEnable_GPIO_Port, motorBackwardEnable_Pin, 0);
		//HAL_GPIO_WritePin(motorForwardEnable_GPIO_Port, motorForwardEnable_Pin, 1);
		if(motor->desired.direction != encoder->measuredDirection_bool) {
		for(int a = motor->pwmValue_u16 ; a > 0; a -= 3)  {
			__HAL_TIM_SET_COMPARE(&htim3, motorBackward_Pin, a);
			HAL_Delay(1);
		}
	}
		motorPin = motorForward_Pin;
	}
	
	__HAL_TIM_SET_COMPARE(&htim3, motorBackward_Pin, motor->pwmValue_u16);
	motor->pwmLastValue_u16 = motor->pwmValue_u16;
}
