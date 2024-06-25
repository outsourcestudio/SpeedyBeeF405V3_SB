/*
 * timer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "timer.h"

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

void (*func_cb)(void);

bool timerInit()
{
	bool ret = true;

	TIM_ClockConfigTypeDef sClockSourceConfig1 = {0};
	TIM_MasterConfigTypeDef sMasterConfig1 = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 83;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
	Error_Handler();
	}
	sClockSourceConfig1.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_TIM_Base_Init(&htim5);
	HAL_TIM_Base_Start(&htim5);

	/////////////////////////////////////////////////////
	//STEP MOTOR TIMER//
	func_cb = NULL;
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

	return ret;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM6)
  {
    if (func_cb != NULL)
    {
      (*func_cb)();
    }
  }
}

void timAttachInterrupt(void (*func)())
{
  func_cb = func;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM5)
    {
      /* USER CODE BEGIN TIM5_MspInit 0 */

      /* USER CODE END TIM5_MspInit 0 */
      /* TIM5 clock enable */
      __HAL_RCC_TIM5_CLK_ENABLE();

      /* TIM5 interrupt Init */
      HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(TIM5_IRQn);
      /* USER CODE BEGIN TIM5_MspInit 1 */

      /* USER CODE END TIM5_MspInit 1 */
    }
  else if(tim_baseHandle->Instance==TIM6)
    {
    /* USER CODE BEGIN TIM6_MspInit 0 */

    /* USER CODE END TIM6_MspInit 0 */
      /* TIM6 clock enable */
      __HAL_RCC_TIM6_CLK_ENABLE();

      /* TIM6 interrupt Init */
      HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    /* USER CODE BEGIN TIM6_MspInit 1 */

    /* USER CODE END TIM6_MspInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM5)
  {
    /* USER CODE BEGIN TIM5_MspDeInit 0 */

    /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
    /* USER CODE BEGIN TIM5_MspDeInit 1 */

    /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}
