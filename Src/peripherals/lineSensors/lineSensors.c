/**************************************************************************/
/*!
    @file    lineSensor.c
    @author   PLF Pacabot.com
    @date     01 December 2014
    @version  0.10
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "config/errors.h"
#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/lineSensors/lineSensors.h"

#define FREQ_TELEMETERS_DIVIDER (1) //freq telemeters = 50KHz/DIVIDER

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

__IO uint16_t ADC1ConvertedValues[2] = {0};
__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void LineSensors_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig;

	    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	    */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	  hadc1.Init.Resolution = ADC_RESOLUTION12b;
	  hadc1.Init.ScanConvMode = ENABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.DiscontinuousConvMode = ENABLE;
	  hadc1.Init.NbrOfDiscConversion = 2;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 2;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.EOCSelection = EOC_SEQ_CONV;
	  HAL_ADC_Init(&hadc1);

	  hadc3.Instance = ADC3;
	  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	  hadc3.Init.Resolution = ADC_RESOLUTION12b;
	  hadc3.Init.ScanConvMode = ENABLE;
	  hadc3.Init.ContinuousConvMode = DISABLE;
	  hadc3.Init.DiscontinuousConvMode = ENABLE;
	  hadc3.Init.NbrOfDiscConversion = 3;
	  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc3.Init.NbrOfConversion = 3;
	  hadc3.Init.DMAContinuousRequests = DISABLE;
	  hadc3.Init.EOCSelection = EOC_SEQ_CONV;
	  HAL_ADC_Init(&hadc3);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_LEFT_EXT;  // Line sensor extreme left
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_RIGHT_EXT; // Line sensor extreme right
	  sConfig.Rank = 2;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_LEFT; // Line sensor left
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_FRONT; // Line sensor front
	  sConfig.Rank = 2;
	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	    */
	  sConfig.Channel = RX_RIGHT; // Line sensor right
	  sConfig.Rank = 3;
	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

	  // -- Enables ADC DMA request
	  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK)
	  {

	  }
	  // -- Enables ADC DMA request
	  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3ConvertedValues, 3) != HAL_OK)
	  {

	  }
}

void LineSensors_Start(void)
{
//	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
//	  /* Start Channel1 */
//	  if(HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
//	  {
//	    /* Starting Error */
//	    //Error_Handler();
//	  }
}

void LineSensors_Stop(void)
{
//	  /*##-2- Stop the TIM Base generation in interrupt mode ####################*/
//	  /* Start Channel1 */
//	  if(HAL_TIM_Base_Stop_IT(&htim10) != HAL_OK)
//	  {
//	    /* Starting Error */
//	    //Error_Handler();
//	  }
	  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
	  lineSensors.active_state = FALSE;
}

void LineSensors_Calibrate(void)
{
//      Telemeters_Start();
}

//void LineSensors_Get_Values(struct telemeters *telemeters)
//{
////      Telemeters_Start();
//}
//
//void LineSensors_Get_Wall(struct telemeters *telemeters)
//{
////      Telemeters_Start();
//}

void LineSensors_IT(void)
{
	  static char selector;

	  switch (selector)
	  {
	  case 0:
		  lineSensors.active_state = TRUE;
		  HAL_ADC_Stop_DMA(&hadc1);
		  HAL_ADC_Stop_DMA(&hadc3);
		  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, SET);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, 2);
		  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3ConvertedValues, 3);
		break;
      case 1:
    	  lineSensors.left_ext.adc_value  	= ADC1ConvertedValues[0];
    	  lineSensors.right_ext.adc_value 	= ADC1ConvertedValues[1];
    	  lineSensors.left.adc_value 		= ADC3ConvertedValues[0];
    	  lineSensors.front.adc_value 		= ADC3ConvertedValues[1];
    	  lineSensors.right.adc_value 		= ADC3ConvertedValues[2];

    	  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);

		  HAL_ADC_Stop_DMA(&hadc1);
		  HAL_ADC_Stop_DMA(&hadc3);
	    break;
	  }

	  selector++;
	  if (selector > ((2 * FREQ_TELEMETERS_DIVIDER)-1))    //freq telemeters = 10Khz/DIVIDER (20Khz/2 => 10Khz)
		  selector = 0;
}

