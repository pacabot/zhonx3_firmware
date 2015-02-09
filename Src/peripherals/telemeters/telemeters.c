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

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;

__IO uint16_t ADC2ConvertedValues[4] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

/*## ADC Telemeters callback for distance computing  #############################*/
	  /* ------------------------------------------------------------------------
	    PA5	ADC2_IN11	R_DIAG_RX		ADC_REGULAR_RANK_1
	    PA6	ADC2_IN6	L_FRONT_RX		ADC_REGULAR_RANK_2
		PA6	ADC2_IN10	R_FRONT_RX		ADC_REGULAR_RANK_3
		PA5	ADC2_IN5	L_DIAG_RX		ADC_REGULAR_RANK_4
	     ------------------------------------------------------------------------ */

void telemetersInit(void)
{
	ADC_ChannelConfTypeDef sConfig;
	ADC_InjectionConfTypeDef sConfigInjected;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc2);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 4;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
	sConfigInjected.InjectedRank = 2;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
	sConfigInjected.InjectedRank = 3;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
	sConfigInjected.InjectedRank = 4;
	HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

	telemeters.it_cnt = 0;
	telemeters.end_of_conversion = 0;

	telemetersStart();
}

void telemetersStart(void)
{
	telemeters.emitter_state = TRUE;
	telemeters.selector = 0;
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	telemeters.active_state = TRUE;
//	Telemeters_Calibrate();
}

void telemetersStop(void)
{
	HAL_ADCEx_InjectedStop(&hadc2);

	HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	telemeters.active_state = FALSE;
}

void telemetersCalibrate(void)
{
	uint32_t nb_conversion = 0;

	for (nb_conversion = 0; nb_conversion < 1000; nb_conversion++)
	{
		telemeters.left_front.offset 	+= telemeters.left_front.adc_value;
		telemeters.left_diag.offset 	+= telemeters.left_diag.adc_value;
		telemeters.right_diag.offset 	+= telemeters.right_diag.adc_value;
		telemeters.right_front.offset 	+= telemeters.right_front.adc_value;
		HAL_Delay(1);
	}

	telemeters.left_front.offset 	/= nb_conversion++;
	telemeters.left_diag.offset 	/= nb_conversion++;
	telemeters.right_diag.offset 	/= nb_conversion++;
	telemeters.right_front.offset 	/= nb_conversion++;
}

//void telemetersGetValues(struct telemeters *telemeters)
//{
////      Telemeters_Start();
//}
//
//void telemetersGetWall(struct telemeters *telemeters)
//{
////      Telemeters_Start();
//}


void telemeters_IT(void)
{
	switch (telemeters.selector)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, SET);
		telemeters.right_front.emitter_state = 1;
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, SET);
		telemeters.left_front.emitter_state = 1;
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		telemeters.left_diag.emitter_state = 1;
		break;
	}
	telemeters.it_cnt++;
	telemeters.selector++;
	if (telemeters.selector > 3)
		telemeters.selector = 0;
	HAL_ADCEx_InjectedStart_IT(&hadc2);
}

void telemeters_INJECTED_ADC_IT(void)
{

	if (telemeters.right_front.emitter_state == 1)
	{
	telemeters.right_front.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
	telemeters.right_front.telemeter_value = telemeters.right_front.adc_value - telemeters.right_front.offset;
	HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
	telemeters.right_front.emitter_state = 0;
	}
	if (telemeters.left_front.emitter_state == 1)
	{
	telemeters.left_front.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
	telemeters.left_front.telemeter_value = telemeters.left_front.adc_value - telemeters.left_front.offset;
	HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
	telemeters.left_front.emitter_state = 0;
	}
	if (telemeters.left_diag.emitter_state == 1)
	{
	telemeters.right_diag.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	telemeters.right_diag.telemeter_value = telemeters.right_diag.adc_value - telemeters.right_diag.offset;
	telemeters.left_diag.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	telemeters.left_diag.telemeter_value = telemeters.left_diag.adc_value - telemeters.left_diag.offset;
	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	telemeters.left_diag.emitter_state = 0;
	}
	telemeters.end_of_conversion++;
}

void telemetersTest(void)
{
	telemetersInit();

	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10, 0,  "LFRONT  = ", (int32_t) telemeters.left_front.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 9,  "LDIAG   = ", (int32_t) telemeters.left_diag.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 18, "RDIAG   = ", (int32_t) telemeters.right_diag.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 27, "RFRONT  = ", (int32_t) telemeters.right_front.telemeter_value, &Font_5x8);

		ssd1306PrintInt(10, 47, "interrupt cnt =  ", (int32_t) telemeters.it_cnt/1000, &Font_5x8);
		ssd1306PrintInt(10, 57, "end of conv.  =  ", (int32_t) telemeters.end_of_conversion/1000, &Font_5x8);
		ssd1306Refresh();
	}
	antiBounceJoystick();
	telemetersStop();
}
