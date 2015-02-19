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

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

//__IO uint16_t ADC1ConvertedValues[2] = {0};
__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */

/**************************************************************************/
/*!
    RANK 1		ADC3  CHANNEL 3 				RX_LEFT_EXT
    RANK 2		ADC4  CHANNEL 2 				RX_LEFT
    RANK 2		ADC1  CHANNEL 3 				RX_FRONT
    RANK 1		ADC13 CHANNEL 2 				RX_RIGHT
    RANK 3		ADC12 CHANNEL 3 				RX_RIGHT_EXT
 */
/**************************************************************************/
void lineSensorsInit(void)
{
//	ADC_ChannelConfTypeDef sConfig;
	ADC_InjectionConfTypeDef sConfigInjected;

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = RX_LEFT;
	sConfigInjected.InjectedRank = 2;
	sConfigInjected.InjectedNbrOfConversion = 2;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = RX_RIGHT;
	sConfigInjected.InjectedRank = 1;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = RX_LEFT_EXT;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 3;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = RX_FRONT;
	sConfigInjected.InjectedRank = 2;
	HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = RX_RIGHT_EXT;
	sConfigInjected.InjectedRank = 3;
	HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

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
}

void lineSensorsStart(void)
{
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc3);
}

void lineSensorsCalibrate(void)
{
	//      Telemeters_Start();
}

void lineSensors_IT(void)
{
	static char selector;

	switch (selector)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, SET);
		break;
	case 1:
		HAL_ADCEx_InjectedStart_IT(&hadc2);
		HAL_ADCEx_InjectedStart_IT(&hadc3);
		break;
	}

	selector++;
	if (selector > ((2 * FREQ_TELEMETERS_DIVIDER)-1))    //freq telemeters = 10Khz/DIVIDER (20Khz/2 => 10Khz)
		selector = 0;

	lineSensors.active_ADC2		=TRUE;
	lineSensors.active_ADC3		=TRUE;
	lineSensors.emitter_state	=TRUE;
}

void lineSensors_ADC_IT(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc2)
	{
		lineSensors.right.adc_value 		= HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		lineSensors.left.adc_value 			= HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
		lineSensors.active_ADC2	= FALSE;
	}
	else
	{
		lineSensors.left_ext.adc_value  	= HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
		lineSensors.front.adc_value 		= HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
		lineSensors.right_ext.adc_value 	= HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);
		lineSensors.active_ADC3	= FALSE;
	}
	if (lineSensors.active_ADC2 == FALSE && lineSensors.active_ADC3 == FALSE )
	{
		HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
		lineSensors.emitter_state	=FALSE;
	}
}

void lineSensorsTest(void)
{

	lineSensorsInit();
	lineSensorsStart();

	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 25, "FRONT     =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);
		ssd1306Refresh();
	}

}

