
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_uart_lib.h"

/* Extern variables -----------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern RAKE_UART_HandleTypeDef ruart1;

/* Private functions -----------------------------------------------*/

/*
	Uart structtaki degiskenlere deger atama islemi burada yapildi .
	!!!! char tipindeki bazi degiskenlere deger atanmadi !!!!
*/
void RAKE_UART_Init(void) {
	ruart1.txBufferLen = 0;
}


void RAKE_Rx_Motor_Speed(MOTOR_HandleTypeDef *motor, FLAG_HandleTypeDef *flag, RAKE_UART_HandleTypeDef *uart) {
	if(flag->UART.rxComplete_bool == 1) {
		if(uart->rxBuffer[5] == 'C') {
			//----------------------------------------
			//Convert Text Data to Integer Code
			//----------------------------------------
		 motor->desired.RPM_f32 	= 
															(uart->rxBuffer[2] - '0') * 100 +
															(uart->rxBuffer[3] - '0') * 10 +
															(uart->rxBuffer[4] - '0');
			motor->desired.direction = 
															(uart->rxBuffer[1] - '0');
			//----------------------------------------
			motor->desired.PWM_u16 = (uint16_t)RAKE_Convert(RPM_TO_PWM, motor->desired.RPM_f32);
			flag->UART.rxComplete_bool = 0;
		}
	}
}

void RAKE_Tx_Motor_Speed(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder, RAKE_UART_HandleTypeDef *uart) {
	if(timer->communicationUART_u16 > TX_TIME){
		/*
			Data Type --> "SavvvCF"
			S = Start								(char)
			a = measured.direction 	(integer)
			vvv = measured.RPM			(char[3])
			C = Control							(char)
			F = Finish							(char)
		*/
		//----------------------------------------
		//Send Data Code
		//----------------------------------------
		int hun = 0, ten = 0, one = 0;
		uint16_t motorSpeed = encoder->measuredSpeed_f32;
		int motorDirection = encoder->measuredDirection_bool; 
		hun = (motorSpeed / 100);
		ten	= (motorSpeed % 100) / 10;
		one	= (motorSpeed % 10);
		
		uart->txBufferLen = sprintf(uart->txBuffer, "S%d%d%d%dCF",             // !!!! burada bir warning var buna bakilmasi gerek !!!!
 															 motorDirection, hun, ten, one);
		
		HAL_UART_Transmit_IT(&huart1, uart->txBuffer, uart->txBufferLen);
		timer->communicationUART_u16 = 0;
	}
}




