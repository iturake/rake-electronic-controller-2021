
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_timer_lib.h"

/* Private variables -----------------------------------------------*/

TIM_HandleTypeDef rake_htim2;
TIM_HandleTypeDef rake_htim3;
TIM_HandleTypeDef rake_htim4;

/* Private structures -----------------------------------------------*/

struct timerStruct timer = {0,0,0,0,0,0};

/* Private functions -----------------------------------------------*/

void RAKE_TIM2_Init(void)  // bunlari çekmeye gerek var miydi reis bilmiyorum ama is yapar
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  rake_htim2.Instance = TIM2;
  rake_htim2.Init.Prescaler = 0;
  rake_htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  rake_htim2.Init.Period = 65535;
  rake_htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  rake_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&rake_htim2, &sConfig) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&rake_htim2, &sMasterConfig) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

void RAKE_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  rake_htim3.Instance = TIM3;
  rake_htim3.Init.Prescaler = 7;
  rake_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  rake_htim3.Init.Period = 999;
  rake_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  rake_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&rake_htim3) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&rake_htim3, &sMasterConfig) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&rake_htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&rake_htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  RAKE_TIM_MspPostInit(&rake_htim3);

}

void RAKE_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  rake_htim4.Instance = TIM4;
  rake_htim4.Init.Prescaler = 7;
  rake_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  rake_htim4.Init.Period = 999;
  rake_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  rake_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&rake_htim4) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&rake_htim4, &sClockSourceConfig) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&rake_htim4, &sMasterConfig) != HAL_OK)
  {
    RAKE_Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

void RAKE_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if(htim->Instance == TIM4) {
			timer.slowStartMotor_u16++;
			timer.communicationUART_u16++;
			timer.pidCalculator_u16++;
			timer.velocityCalculator_u16++;
			timer.ledDriver_u16++;
		}
}

void RAKE(TIM_HandleTypeDef *htim) {
	UNUSED(htim);
}


