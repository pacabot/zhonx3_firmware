/**************************************************************************/
/*!
    @file    TimeBase.c
    @author  PLF (PACABOT)
    @date    03 August 2014
    @version  0.10
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/telemeters/telemeters.h"
#include "controls/pid/pid.h"

#include "times_base/times_base.h"
#include "config/config.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

GPIO_InitTypeDef GPIO_InitStruct;

volatile int32_t Blink[3] = {500, 500, 0};

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void TimesBase_Init(void)
{
	  TIM_MasterConfigTypeDef sMasterConfig;
	  uint32_t uwPrescalerValue = 0;
	  /*##-1- Configure the TIM peripheral #######################################*/
	  /* -----------------------------------------------------------------------
	    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	    since APB1 prescaler is different from 1.
	      TIM3CLK = 2 * PCLK1
	      PCLK1 = HCLK / 4
	      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
	    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
	    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1
	      ----------------------------------------------------------------------- */

	  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (HI_TIME_FREQ * 2)) - 1;
	  htim7.Instance = TIM7;
	  htim7.Init.Prescaler =  uwPrescalerValue;
	  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim7.Init.Period = 2-1;
	  htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  HAL_TIM_Base_Init(&htim7);

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
	  /* Start Channel1 */
	  if(HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	  {
	    /* Starting Error */
//	    Error_Handler();
	  }

	  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (LOW_TIME_FREQ * 100)) - 1;

	  htim6.Instance = TIM6;
	  htim6.Init.Prescaler =  uwPrescalerValue;
	  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim6.Init.Period = 100-1;
	  HAL_TIM_Base_Init(&htim6);

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
	  /* Start Channel1 */
	  if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
	  {
	    /* Starting Error */
//	    Error_Handler();
	  }
}

void Led_Power_Blink(unsigned int off_time, unsigned int on_time, unsigned int repeat)
{
      Blink[0] = (off_time / 10);
      Blink[1] = (on_time / 10);
      Blink[2] = (repeat / 10);
}

void Led_Blink_IT(void)
{
	  static unsigned int cnt_led = 0;

	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  cnt_led++;
	  if(cnt_led <= (Blink[0]))
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
	  }
	  if(cnt_led >= ((Blink[0]+Blink[1])))
		  cnt_led = 0;
}

void High_Freq_IT(void)
{
	Pids_IT();
	ADXRS620_IT();
//	Telemeters_IT();
}

void Low_Freq_IT(void)
{
	Led_Blink_IT();
}

