/**
  ******************************************************************************
  * @file    stm32f10x_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32F1 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32F10x_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_adc.h"
/* USER CODE BEGIN Includes */
#include "product.h"
#include "task_cli.h"


/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void HardFault_Handler(void);
void EXTI15_10_IRQHandler (void);
#include "mcpwm_foc.h"

/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

  //TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  	  mcpwm_foc_adc_int_handler((void*)0, 0);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */

}

void ADC3_IRQHandler(void){

}

void WWDG_IRQHandler(void){

}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */
    LL_TIM_ClearFlag_UPDATE(TIM1);
    //R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);

   /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

   /* USER CODE END TIMx_UP_M1_IRQn 1 */
}

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  //if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
  //{
    //LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);

    //R3_2_BRK_IRQHandler(&PWM_Handle_M1);

  //}
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/


  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */


  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

/**
  * @brief  This function handles Button IRQ on PIN PC15.
  */
void EXTI15_10_IRQHandler (void)
{
	/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_15) )
  {
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_15);
    //UI_HandleStartStopButton_cb ();
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
