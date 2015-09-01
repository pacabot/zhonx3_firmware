/**************************************************************************/
/*!
    @file    telemeters.c
    @author  PLF (PACABOT)
    @date    11 July 2015
    @version 1.0
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

telemetersStruct telemeters;

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
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
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
	hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
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

	memset(&telemeters.FR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.FL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.DR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.DL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters, 0, sizeof(telemetersStruct));

	telemeters.FL.mm_conv.old_avrg = 0;
	telemeters.FR.mm_conv.old_avrg = 0;
	telemeters.DL.mm_conv.old_avrg = 0;
	telemeters.DR.mm_conv.old_avrg = 0;

	telemeters.FL.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.DL.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.FR.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.DR.mm_conv.cell_idx = NUMBER_OF_CELL - 1;

	telemeters.FL.mm_conv.profile = telemeter_FL_profile;
	telemeters.DL.mm_conv.profile = telemeter_DL_profile;
	telemeters.FR.mm_conv.profile = telemeter_FR_profile;
	telemeters.DR.mm_conv.profile = telemeter_DR_profile;

	telemeters.FL.led_gpio = TX_FL;
	telemeters.DL.led_gpio = TX_DUAL_DIAG;
	telemeters.FR.led_gpio = TX_FR;
	telemeters.DR.led_gpio = TX_DUAL_DIAG;
}

void telemetersStart(void)
{
	telemeters.active_state = TRUE;
}

void telemetersStop(void)
{
	HAL_GPIO_WritePin(GPIOB, TX_FL, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_FR, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	telemeters.active_state = FALSE;
}

void telemetersAdc2Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start_IT(&hadc2);
	telemeters.it_cnt++;
}

void telemetersAdc3Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	HAL_ADC_Start_IT(&hadc3);
}

void telemeters_IT(void)
{
	telemeters.selector++;

	if (telemeters.selector > 10)
	{
		telemeters.selector = 0;
		HAL_GPIO_WritePin(GPIOB, TX_FR, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_FL, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.FR.isActivated = TX_OFF;
		telemeters.FL.isActivated = TX_OFF;
		telemeters.DL.isActivated = TX_OFF;
		telemeters.DR.isActivated = TX_OFF;
	}

	switch (telemeters.selector)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOB, TX_FL, SET);
		return;
	case 2:
		telemeters.FL.isActivated = TX_ON;
		sConfig.Channel = RX_FL;
		telemetersAdc2Start();
		return;
	case 3:
		HAL_GPIO_WritePin(GPIOB, TX_FR, SET);
		return;
	case 4:
		telemeters.FR.isActivated = TX_ON;
		sConfig.Channel = RX_FR;
		telemetersAdc2Start();
		return;
	case 5:
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		return;
	case 6:
		telemeters.DL.isActivated = TX_ON;
		sConfig.Channel = RX_DL;
		telemetersAdc2Start();
		telemeters.DR.isActivated = TX_ON;
		sConfig.Channel = RX_DR;
		telemetersAdc3Start();
		return;
	case 7:
		return;
	case 8:
		telemeters.FR.isActivated = TX_OFF;
		sConfig.Channel = RX_FR;
		telemetersAdc2Start();
		return;
	case 9:
		telemeters.FL.isActivated = TX_OFF;
		sConfig.Channel = RX_FL;
		telemetersAdc2Start();
		return;
	case 10:
		telemeters.DL.isActivated = TX_OFF;
		sConfig.Channel = RX_DL;
		telemetersAdc2Start();
		telemeters.DR.isActivated = TX_OFF;
		sConfig.Channel = RX_DR;
		telemetersAdc3Start();
		return;
	}
}

void telemeters_ADC2_IT(void)
{
	if (telemeters.FL.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.FL, &hadc2);
		getTelemetersDistance(&telemeters.FL);
		goto end;
	}
	if (telemeters.FR.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.FR, &hadc2);
		getTelemetersDistance(&telemeters.FR);
		goto end;
	}
	if (telemeters.DL.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.DL, &hadc2);
		getTelemetersDistance(&telemeters.DL);
		goto end;
	}

end :
	telemeters.end_of_conversion++;
}

void telemeters_ADC3_IT(void)
{
	if (telemeters.DR.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.DR, &hadc3);
		getTelemetersDistance(&telemeters.DR);
	}
}

void getTelemetersADC(telemeterStruct *tel, ADC_HandleTypeDef *hadc)
{
	if (tel->isActivated == TX_ON)
	{
		tel->adc = HAL_ADC_GetValue(hadc);
		HAL_GPIO_WritePin(GPIOB, tel->led_gpio, RESET);

		if (tel->adc - tel->avrg_ref > 0)
			tel->avrg = mobileAvrgInt(&tel->mAvrgStruct, (tel->adc - tel->avrg_ref));
		else
			tel->avrg = 0;

		tel->isActivated = FALSE;
	}
	if (tel->isActivated == TX_OFF)
	{
		tel->adc_ref = HAL_ADC_GetValue(hadc);
		tel->avrg_ref = mobileAvrgInt(&tel->mAvrgStruct_ref, tel->adc_ref);

		tel->isActivated = FALSE;
	}
}

/*
 * Formulas
 * y=ax+b
 *
 * 	  yb-ya
 * a=_______
 *    xb-xa
 *
 * b=y-ax
 *
 * 		  yb-ya
 * b=ya- ________
 * 		  xb-xa
 */

void getTelemetersDistance(telemeterStruct *tel)
{
	char sens = 1;

	if(tel->avrg > tel->mm_conv.old_avrg)
	{
		sens = -1;
	}
	else if(tel->avrg == tel->mm_conv.old_avrg)	//for optimize redundant call
	{
		return;
	}

	while ((tel->avrg > tel->mm_conv.profile[tel->mm_conv.cell_idx]) || (tel->avrg < tel->mm_conv.profile[tel->mm_conv.cell_idx + 1]))
	{
		tel->mm_conv.cell_idx += sens;
		if (tel->mm_conv.cell_idx < 0)
		{
			tel->mm_conv.cell_idx = 0;
			break;
		}
		else if (tel->mm_conv.cell_idx >= NUMBER_OF_CELL)
		{
			tel->mm_conv.cell_idx = NUMBER_OF_CELL;
			break;
		}
	}

	/*
	 * 		(ya xb - xa yb - yc xb + yc xa)
	 * xc= _________________________________
	 * 				  (-yb + ya)
	 * xc <- distance in millimeters
	 * yc <- voltage measured
	 *
	 * xa <- distance in millimeters measured in the calibrate function
	 * ya <- voltage measure in the calibrate function, the voltage correspond to the distance of xa
	 *
	 * xb <- distance in millimeters measured in the calibrate function, it's the distance xa+DISTANCE_BY_LOOP
	 * yb <- voltage measure in the calibrate function, the voltage correspond to the distance of xb
	 *
	 * XXXX is the sensor reference : front_left , front_right , diag_right , diag_left
	 *
	 *
	 * 		telemeter_XXXX_voltage[cell_XXXX] (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP telemeter_XXXX_voltage[cell_XXXX + 1] - value_XXX (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_XXXX cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP
	 * xc= _______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
	 * 																		-telemeter_XXX_voltage[cell_XXXX + 1] + telemeter_XXX_voltage[cell_XXXX]
	 *
	 */
	//TODO : conversion to mm
}

int getTelemetersVariation(telemeterStruct *tel)
{
	getTelemetersDistance(tel);
	return tel->delta_mm;
}

void telemetersTest(void)
{
	char joy;
	telemetersInit();
	telemetersStart();

	while (joy != JOY_LEFT)	//todo make a generic test menu (unit test)
	{
		joy = expanderJoyFiltered();
		ssd1306ClearScreen();

		ssd1306DrawString(1,0, "   AVRG 1/10mm", &Font_5x8);

		ssd1306PrintInt(1, 9 , "FL ", (int32_t) telemeters.FL.avrg, &Font_5x8);
		ssd1306PrintInt(1, 18, "DL ", (int32_t) telemeters.DL.avrg, &Font_5x8);
		ssd1306PrintInt(1, 27, "DR ", (int32_t) telemeters.DR.avrg, &Font_5x8);
		ssd1306PrintInt(1, 36, "FR ", (int32_t) telemeters.FR.avrg, &Font_5x8);

		ssd1306PrintInt(45, 9 ,"", (int32_t) (telemeters.FL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 18,"", (int32_t) (telemeters.DL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 27,"", (int32_t) (telemeters.DR.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 36,"", (int32_t) (telemeters.FR.dist_mm * 10), &Font_5x8);

//		ssd1306PrintInt(1, 45, "IT TIME  =  ", (int32_t) telemeters.it_time, &Font_5x8);
		ssd1306PrintInt(1, 54, "IT ERROR =  ", (telemeters.end_of_conversion - telemeters.it_cnt), &Font_5x8);
		ssd1306Refresh();

		if (joy == JOY_RIGHT)
		{
			while (joy != JOY_LEFT)
			{
				joy = expanderJoyFiltered();
				ssd1306ClearScreen();
				ssd1306DrawString(1,0, "   ADC  REF  VAR", &Font_5x8);

				ssd1306PrintInt(1, 9 , "FL ", (int32_t) telemeters.FL.adc, &Font_5x8);
				ssd1306PrintInt(1, 18, "DL ", (int32_t) telemeters.DL.adc, &Font_5x8);
				ssd1306PrintInt(1, 27, "DR ", (int32_t) telemeters.DR.adc, &Font_5x8);
				ssd1306PrintInt(1, 36, "FR ", (int32_t) telemeters.FR.adc, &Font_5x8);

				ssd1306PrintInt(45, 9 , "", (int32_t) telemeters.FL.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 18, "", (int32_t) telemeters.DL.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 27, "", (int32_t) telemeters.DR.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 36, "", (int32_t) telemeters.FR.avrg_ref, &Font_5x8);

//				ssd1306PrintInt(75, 9 , "", (int32_t) (telemeters.FL.delta_avrg), &Font_5x8);
//				ssd1306PrintInt(75, 18, "", (int32_t) (telemeters.DL.delta_avrg), &Font_5x8);
//				ssd1306PrintInt(75, 27, "", (int32_t) telemeters.DR.delta_mm_avrg, &Font_5x8);
//				ssd1306PrintInt(75, 36, "", (int32_t) telemeters.FR.delta_mm_avrg, &Font_5x8);

				ssd1306PrintInt(75, 9 , "", (int32_t) telemeters.FL.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 18, "", (int32_t) telemeters.DL.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 27, "", (int32_t) telemeters.DR.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 36, "", (int32_t) telemeters.FR.speed_mms, &Font_5x8);

				ssd1306Refresh();
			}
			while (joy == JOY_LEFT)
			{
				joy = expanderJoyFiltered();
			}
		}
	}
	antiBounceJoystick();
	telemetersStop();
}
