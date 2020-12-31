
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_uart_lib.h"
#include "rake_stm32_extra_lib.h"
#include "rake_stm32_motor_lib.h"
#include "rake_stm32_timer_lib.h"
#include "rake_stm32_encoder_lib.h"

/* Private structures -----------------------------------------------*/

struct uartStruct uart = {0,0,0,0};
struct flagStruct flag = {{0,0,0,0,0,0,0,0},{0,0}};

/* Private variables -----------------------------------------------*/

UART_HandleTypeDef rake_huart1;


/* Private functions -----------------------------------------------*/

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

void RAKE_Tx_Motor_Speed(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder) {
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
		
		uart.txBufferLen = sprintf(uart.txBuffer, "S%d%d%d%dCF",
															 motorDirection, hun, ten, one);
		
		HAL_UART_Transmit_IT(&rake_huart1, uart.txBuffer, uart.txBufferLen);
		timer->communicationUART_u16 = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1){
		flag.LED.UART_bit = 1;
		//----------------------------------------
		//Reset RX Buffer Code
		//----------------------------------------
		if(flag.UART.rxIndex_bool == 0) {
			for(uint8_t index; index < 30; index++) {
				uart.rxBuffer[index] = 0;
			}
		}
		
		if(uart.rxData[0] == 'S') {
			flag.UART.rxIndex_bool = 0;
		}
		//----------------------------------------
		//Write Data to Buffer Code
		//----------------------------------------
		if(uart.rxData[0] != 'F'){
			uart.rxBuffer[flag.UART.rxIndex_bool++] = uart.rxData[0];
		} else {
			flag.UART.rxIndex_bool = 0;
			flag.UART.rxComplete_bool = 1;
		}
		//----------------------------------------
		HAL_UART_Receive_IT(&rake_huart1, uart.rxData, 1);
		//----------------------------------------
	} else {
		flag.LED.UART_bit = 0;
	}
}






void RAKE_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  rake_huart1.Instance = USART1;
  rake_huart1.Init.BaudRate = 115200;
  rake_huart1.Init.WordLength = UART_WORDLENGTH_8B;
  rake_huart1.Init.StopBits = UART_STOPBITS_1;
  rake_huart1.Init.Parity = UART_PARITY_NONE;
  rake_huart1.Init.Mode = UART_MODE_TX_RX;
  rake_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  rake_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&rake_huart1) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}



