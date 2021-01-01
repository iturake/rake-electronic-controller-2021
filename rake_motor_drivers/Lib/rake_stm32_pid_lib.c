
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_pid_lib.h"

/* Private functions -----------------------------------------------*/

void RAKE_Pid_Calculation(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor, FLAG_HandleTypeDef *flag, RAKE_UART_HandleTypeDef *uart, PID_HandleTypeDef *PID) {
	if(timer->pidCalculator_u16 > PID_TIME) {
		RAKE_Rx_Motor_Speed(motor, flag, uart);
		
		PID->error = motor->desired.RPM_f32 - encoder->measuredSpeed_f32;
		PID->integral = PID->integralPrev + (PID_TIME * (PID->error + PID->lastError) / 2);
		
		PID->derivative = (PID->error - PID->lastError) / PID_TIME;
		PID->output = (PID->error * PID->values.kp) + (PID->derivative * PID->values.kd) + (PID->integral * PID->values.ki);
		
		if(PID->output > VOLTAGE_MAX) {
			PID->output = VOLTAGE_MAX;
			PID->integral = PID->integralPrev;
		} else if(PID->output < -VOLTAGE_MAX) {
			PID->output = -VOLTAGE_MAX;
			PID->integral = PID->integralPrev;
		}
		
		RAKE_Drive_Motor(PID->output, encoder, motor);
		
		PID->lastError = PID->error;
		PID->integralPrev = PID->integral;
		timer->pidCalculator_u16 = 0;
	}
}


