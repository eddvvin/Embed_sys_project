/**
  ******************************************************************************
  * @file    stm32f4xx_it.c (PARTIAL - SysTick Handler Only)
  * @brief   Interrupt Service Routines
  ******************************************************************************
  */

/* ADD THIS TO YOUR INCLUDES SECTION */
#include "rtos.h"



/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  HAL_IncTick();

  /* USER CODE BEGIN SysTick_IRQn 1 */
  RTOS_Tick();

  /* USER CODE END SysTick_IRQn 1 */
}



/**
  * @brief Override HAL's weak SysTick callback
  */
void HAL_SYSTICK_Callback(void)
{
    RTOS_Tick();
}


