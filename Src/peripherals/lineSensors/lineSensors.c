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

extern ADC_HandleTypeDef hadc3;

__IO uint16_t ADC1ConvertedValues[2] = {0};
__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void lineSensorsInit(void)
{
}

void lineSensorsStart(void)
{
}


void lineSensorsStop_DMA_ADC3(void)
{
	  HAL_ADC_Stop_DMA(&hadc3);
	  lineSensors.active_ADC3 = FALSE;

	  if (lineSensors.active_ADC1 == FALSE)
	  {
	  	  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
		  lineSensors.active_state = FALSE;
	  }

	  lineSensors.left_ext.adc_value 	= ADC3ConvertedValues[0];
	  lineSensors.front.adc_value 		= ADC3ConvertedValues[1];
	  lineSensors.right_ext.adc_value 	= ADC3ConvertedValues[2];
}

void lineSensorsCalibrate(void)
{
}

void lineSensors_IT(void)
{
	  static char selector;

	  switch (selector)
	  {
	  case 0:
		  lineSensors.active_state = TRUE;
		  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, SET);
//		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, 2);
		  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3ConvertedValues, 3);
		break;
      case 1:
//    	  HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
//
//		  HAL_ADC_Stop_DMA(&hadc1);
//		  HAL_ADC_Stop_DMA(&hadc3);
	    break;
	  }

	  selector++;
	  if (selector > ((2 * FREQ_TELEMETERS_DIVIDER)-1))    //freq telemeters = 10Khz/DIVIDER (20Khz/2 => 10Khz)
		  selector = 0;
}

