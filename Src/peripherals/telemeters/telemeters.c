/**************************************************************************/
/*!
    @file    ADXRS620.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include "application/statistiques/statistiques.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/telemeters/telemeters.h"

/* extern variables */
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* global variables */
GPIO_InitTypeDef GPIO_InitStruct;
ADC_ChannelConfTypeDef sConfig;

/*## ADC Telemeters callback for distance computing  #############################*/
/* ------------------------------------------------------------------------
	    ADC2_IN11	R_DIAG_RX		ADC_REGULAR_RANK_1
	    ADC2_IN6	L_FRONT_RX		ADC_REGULAR_RANK_2
		ADC2_IN10	R_FRONT_RX		ADC_REGULAR_RANK_3
		ADC2_IN5	L_DIAG_RX		ADC_REGULAR_RANK_4
------------------------------------------------------------------------ */

void telemetersInit(void)
{
	//	ADC_InjectionConfTypeDef sConfigInjected;
	HAL_ADC_Stop_IT(&hadc2);
	HAL_ADC_Stop_IT(&hadc3);

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc2);

	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION12b;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc3);
}

void telemetersStart(void)
{
	telemeters.active_state = TRUE;
	//telemetersCalibrate();
}

void telemetersStop(void)
{
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

void telemetersAdc2Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start_IT(&hadc2);
	telemeters.it_cnt++;
}

void telemetersAdc3Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	HAL_ADC_Start_IT(&hadc3);
}

void telemeters_IT(void)
{
	telemeters.selector++;

	if (telemeters.selector > 9)
	{
		telemeters.selector = 0;
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.right_front.sensor_state = 0;
		telemeters.left_front.sensor_state = 0;
		telemeters.left_diag.sensor_state = 0;
		telemeters.right_diag.sensor_state = 0;
	}

	switch (telemeters.selector)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, SET);
		return;
	case 2:
		telemeters.left_front.sensor_state = 1;
		sConfig.Channel = RX_LEFT_FRONT;
		telemetersAdc2Start();
		return;
	case 3:
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, SET);
		return;
	case 4:
		telemeters.right_front.sensor_state = 1;
		sConfig.Channel = RX_RIGHT_FRONT;
		telemetersAdc2Start();
		return;
	case 5:
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		return;
	case 6:
		telemeters.left_diag.sensor_state = 1;
		sConfig.Channel = RX_LEFT_DIAG;
		telemetersAdc2Start();
		telemeters.right_diag.sensor_state = 1;
		sConfig.Channel = RX_RIGHT_DIAG;
		telemetersAdc3Start();
		return;
	case 7:
		telemeters.ref_right_front.sensor_state = 1;
		sConfig.Channel = RX_RIGHT_FRONT;
		telemetersAdc2Start();
		return;
	case 8:
		telemeters.ref_left_front.sensor_state = 1;
		sConfig.Channel = RX_LEFT_FRONT;
		telemetersAdc2Start();
		return;
	case 9:
		telemeters.ref_left_diag.sensor_state = 1;
		sConfig.Channel = RX_LEFT_DIAG;
		telemetersAdc2Start();
		telemeters.ref_right_diag.sensor_state = 1;
		sConfig.Channel = RX_RIGHT_DIAG;
		telemetersAdc3Start();
		return;
	}
}

void telemeters_ADC2_IT(void)
{
	telemeters.end_of_conversion++;
	if (telemeters.right_front.sensor_state == 1)
	{
		telemeters.right_front.adc_value = HAL_ADC_GetValue(&hadc2);
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
		telemeters.right_front.telemeter_values[(telemeters.end_of_conversion / 8) % SIZE_OF_AVEVAGE_TABLE] = telemeters.right_front.adc_value - telemeters.right_front.offset - telemeters.ref_right_front.adc_value;
		telemeters.right_front.average_value=cAverage((int*)telemeters.right_front.telemeter_values,SIZE_OF_AVEVAGE_TABLE);
		telemeters.right_front.sensor_state = 0;
		return;
	}
	if (telemeters.left_front.sensor_state == 1)
	{
		telemeters.left_front.adc_value = HAL_ADC_GetValue(&hadc2);
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
		telemeters.left_front.telemeter_values[(telemeters.end_of_conversion / 8) % SIZE_OF_AVEVAGE_TABLE] = telemeters.left_front.adc_value - telemeters.left_front.offset - telemeters.ref_left_front.adc_value;
		telemeters.left_front.average_value=cAverage((int*)telemeters.left_front.telemeter_values,SIZE_OF_AVEVAGE_TABLE);
		telemeters.left_front.sensor_state = 0;
		return;
	}
	if (telemeters.left_diag.sensor_state == 1)
	{
		telemeters.left_diag.adc_value = HAL_ADC_GetValue(&hadc2);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.left_diag.telemeter_values[(telemeters.end_of_conversion / 8) % SIZE_OF_AVEVAGE_TABLE] = telemeters.left_diag.adc_value - telemeters.left_diag.offset - telemeters.ref_left_diag.adc_value;
		telemeters.left_diag.average_value=cAverage((int*)telemeters.left_diag.telemeter_values,SIZE_OF_AVEVAGE_TABLE);
		telemeters.left_diag.sensor_state = 0;
		return;
	}
	if (telemeters.ref_left_diag.sensor_state == 1)
	{
		telemeters.ref_left_diag.adc_value = HAL_ADC_GetValue(&hadc2);
		telemeters.ref_left_diag.sensor_state = 0;
		return;
	}
	if (telemeters.ref_left_front.sensor_state == 1)
	{
		telemeters.ref_left_front.adc_value = HAL_ADC_GetValue(&hadc2);
		telemeters.ref_left_front.sensor_state = 0;
		return;
	}
	if (telemeters.ref_right_front.sensor_state == 1)
	{
		telemeters.ref_right_front.adc_value = HAL_ADC_GetValue(&hadc2);
		telemeters.ref_right_front.sensor_state = 0;
		return;
	}
}

void telemeters_ADC3_IT(void)
{
	if (telemeters.right_diag.sensor_state == 1)
	{
		telemeters.right_diag.adc_value = HAL_ADC_GetValue(&hadc3);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.right_diag.telemeter_values[(telemeters.end_of_conversion/8) % SIZE_OF_AVEVAGE_TABLE] = telemeters.right_diag.adc_value - telemeters.right_diag.offset - telemeters.ref_right_diag.adc_value;
		telemeters.right_diag.average_value=cAverage((int*)telemeters.right_diag.telemeter_values,SIZE_OF_AVEVAGE_TABLE);
		telemeters.right_diag.sensor_state = 0;
	}
	if (telemeters.ref_right_diag.sensor_state == 1)
	{
		telemeters.ref_right_diag.adc_value = HAL_ADC_GetValue(&hadc3);
		telemeters.ref_right_diag.sensor_state = 0;
	}
}

void telemetersTest(void)
{
	telemetersInit();
	telemetersStart();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10, 0,  "LFRONT  = ", (int32_t) telemeters.left_front.average_value, &Font_5x8);
		ssd1306PrintInt(10, 9,  "LDIAG   = ", (int32_t) telemeters.left_diag.average_value, &Font_5x8);
		ssd1306PrintInt(10, 18, "RDIAG   = ", (int32_t) telemeters.right_diag.average_value, &Font_5x8);
		ssd1306PrintInt(10, 27, "RFRONT  = ", (int32_t) telemeters.right_front.average_value, &Font_5x8);

//		ssd1306PrintInt(10, 36,  "LFRONT  = ", (int32_t) telemeters.ref_left_front.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 45,  "LDIAG   = ", (int32_t) telemeters.ref_left_diag.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 54, "RDIAG   = ", (int32_t) telemeters.ref_right_diag.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 63, "RFRONT  = ", (int32_t) telemeters.ref_right_front.adc_value, &Font_5x8);

		ssd1306PrintInt(10, 47, "interrupt cnt =  ", (int32_t) telemeters.it_cnt/1000, &Font_5x8);
		ssd1306PrintInt(10, 57, "end of conv.  =  ", (int32_t) telemeters.end_of_conversion/1000, &Font_5x8);
		ssd1306Refresh();
	}
	antiBounceJoystick();
	telemetersStop();
}
