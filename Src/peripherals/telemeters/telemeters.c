/**************************************************************************/
/*!
    @file    ADXRS620.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "config/errors.h"
#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/telemeters/telemeters.h"

#define FREQ_TELEMETERS_DIVIDER (10) //freq telemeters = 50KHz/DIVIDER

extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim10;

__IO uint16_t ADC2ConvertedValues[4] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void Telemeters_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig;

	    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	    */
	  hadc2.Instance = ADC2;
	  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	  hadc2.Init.Resolution = ADC_RESOLUTION12b;
	  hadc2.Init.ScanConvMode = ENABLE;
	  hadc2.Init.ContinuousConvMode = DISABLE;
	  hadc2.Init.DiscontinuousConvMode = ENABLE;
	  hadc2.Init.NbrOfDiscConversion = 4;
	  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc2.Init.NbrOfConversion = 4;
	  hadc2.Init.DMAContinuousRequests = DISABLE;
	  hadc2.Init.EOCSelection = EOC_SEQ_CONV;
	  HAL_ADC_Init(&hadc2);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_LEFT_FRONT;  // IR receiver left front
	  sConfig.Rank = 4;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_RIGHT_DIAG; // IR receive right diag
	  sConfig.Rank = 2;
	  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_RIGHT_FRONT; // IR receive right front
	  sConfig.Rank = 3;
	  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_LEFT_DIAG; // IR receive left diag
	  sConfig.Rank = 1;
	  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

//	  if( HAL_ADC_Start(&hadc2) != HAL_OK)
//	  {
//
//	  }
	  // -- Enables ADC DMA request
	  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, 4) != HAL_OK)
	  {

	  }

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

	  uint32_t uwPrescalerValue = 0;
	  /* Compute the prescaler value to have TIM3 counter clock equal to 100 KHz */

	  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 1000000) - 1;

	  htim10.Instance = TIM10;
	  htim10.Init.Prescaler =  uwPrescalerValue;
	  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim10.Init.Period = 5-1;
	  HAL_TIM_Base_Init(&htim10);
}

void Telemeters_Start(void)
{
	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
	  /* Start Channel1 */
	  if(HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
	  {
	    /* Starting Error */
	    //Error_Handler();
	  }
}

void Telemeters_Stop(void)
{
	  /*##-2- Stop the TIM Base generation in interrupt mode ####################*/
	  /* Start Channel1 */
	  if(HAL_TIM_Base_Stop_IT(&htim10) != HAL_OK)
	  {
	    /* Starting Error */
	    //Error_Handler();
	  }
	  HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
	  HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
	  HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	  telemeters.active_state = FALSE;
}

void Telemeters_Calibrate(void)
{
//      Telemeters_Start();
}

void Telemeters_Get_Values(struct telemeters *telemeters)
{
//      Telemeters_Start();
}

void Telemeters_Get_Wall(struct telemeters *telemeters)
{
//      Telemeters_Start();
}

void Telemeters_IT(void)
{
	  static char selector;

	  switch (selector)
	  {
	  case 0:
		  telemeters.active_state = TRUE;
		  HAL_ADC_Stop_DMA(&hadc2);
		  HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, SET);
		  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, 4);
		break;
      case 1:
    	  telemeters.left_front.adc_value = ADC2ConvertedValues[3];
		  HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);

		  HAL_ADC_Stop_DMA(&hadc2);
		  HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, SET);
		  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, 4);
	    break;
	  case 2:
		  telemeters.right_front.adc_value = ADC2ConvertedValues[2];
		  HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);

		  HAL_ADC_Stop_DMA(&hadc2);
		  HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, 4);
	    break;
	  case 3:
		  telemeters.left_diag.adc_value = ADC2ConvertedValues[0];
		  telemeters.right_diag.adc_value = ADC2ConvertedValues[1];
		  HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		  HAL_ADC_Stop_DMA(&hadc2);
		break;
	  }

	  selector++;
	  if (selector > ((4 * FREQ_TELEMETERS_DIVIDER)-1))    //freq telemeters = 50Khz/DIVIDER (200Khz/4 => 50Khz)
		  selector = 0;
}

