#ifndef RAKE_STM32_EXTRA_LIB_H
#define RAKE_STM32_EXTRA_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "main.h"
#include "rake_stm32_timer_lib.h"

/* Private defines ----------------------------------------------------------*/

#define GEAR_RATIO 1
#define ENCODER_PULS 720

#define VOLTAGE_MAX 12.0
#define RPM_MAX 120.0

#define PWM_MAX 999
#define PWM_ZERO 499
#define PWM_MIN 0

#define TX_TIME 99
#define PID_TIME 9
#define LED_TIME 49
#define VELOCITY_TIME 999

#define SECOND 1000
#define MINUTE 60


/* Private enums ----------------------------------------------------------*/

enum {
	VOLTAGE_TO_PWM,
	PWM_TO_VOLTAGE,
	RPM_TO_PWM,
	PWM_TO_RPM
};

/* Private structures ----------------------------------------------------------*/

typedef struct flagStruct {
	struct {
		_Bool motorForward_bit;
		_Bool motorBackward_bit;
		_Bool UART_bit;
		_Bool CANBUS_bit;
		_Bool adminMode_bit;
		_Bool testMode_bit;
		_Bool normalMode_bit;
		_Bool ERROR_bit;
	} LED;
	struct {
		uint8_t rxIndex_bool;
		_Bool rxComplete_bool;
	} UART;
} FLAG_HandleTypeDef;

/* Private function prototypes -----------------------------------------------*/

float RAKE_Convert(uint8_t convertMode_u8, float convertingValue_f32);
void RAKE_Error_Handler(void);


#ifdef _cplusplus
  }
#endif

#endif
