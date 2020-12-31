#ifndef RAKE_STM32_PID_LIB_H
#define RAKE_STM32_PID_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/



/* Private structures ----------------------------------------------------------*/

struct pidStruct {
	float error;
	float lastError;
	float derivative;
	float integral;
	float integralPrev;
	float output;
	struct {float kp, kd, ki;} values;
};


/* Private function prototypes -----------------------------------------------*/





#ifdef _cplusplus
  }
#endif

#endif
