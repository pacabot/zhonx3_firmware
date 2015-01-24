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

__IO uint16_t ADC2ConvertedValues[4] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

/*## ADC Telemeters callback for distance computing  #############################*/
	  /* ------------------------------------------------------------------------
	    PA5	ADC2_IN11	R_DIAG_RX		ADC_REGULAR_RANK_1
	    PA6	ADC2_IN6	L_FRONT_RX		ADC_REGULAR_RANK_2
		PA6	ADC2_IN10	R_FRONT_RX		ADC_REGULAR_RANK_3
		PA5	ADC2_IN5	L_DIAG_RX		ADC_REGULAR_RANK_4
	     ------------------------------------------------------------------------ */

void Telemeters_Init(void)
{
	telemeters.it_cnt = 0;
	telemeters.end_of_conversion = 0;
	Telemeters_Start();
}

void Telemeters_Start(void)
{
	telemeters.emitter_state = TRUE;
	telemeters.selector = 0;
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, 4);
//	Telemeters_Calibrate();
}

void Telemeters_Stop(void)
{
	HAL_ADC_Stop_DMA(&hadc2);

	HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	telemeters.active_state = FALSE;
}

void Telemeters_Calibrate(void)
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

//	telemeters.selector = (__IO uint32_t) &hadc2.NbrOfCurrentConversionRank;
	switch (telemeters.selector)
	{
	case 0:
		telemeters.it_cnt++;
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, SET);
		break;
	case 1:
		telemeters.right_front.adc_value = ADC2ConvertedValues[0];
		telemeters.right_front.telemeter_value = telemeters.right_front.adc_value - telemeters.right_front.offset;
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, SET);
		break;
	case 5:
		telemeters.left_front.adc_value = ADC2ConvertedValues[1];
		telemeters.left_front.telemeter_value = telemeters.left_front.adc_value - telemeters.left_front.offset;
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		break;
	case 9:
		telemeters.right_diag.adc_value = ADC2ConvertedValues[2];
		telemeters.right_diag.telemeter_value = telemeters.right_diag.adc_value - telemeters.right_diag.offset;
		telemeters.left_diag.adc_value = ADC2ConvertedValues[3];
		telemeters.left_diag.telemeter_value = telemeters.left_diag.adc_value - telemeters.left_diag.offset;
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		break;
	}

	telemeters.selector++;
	if (telemeters.selector > 9)
		telemeters.selector = 0;

}

void Telemeters_REGULAR_ADC_IT(void)
{
//	telemeters.selector = (__IO uint32_t) &hadc2.NbrOfCurrentConversionRank;

//	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
//	HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
//	HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);

	if (telemeters.right_front.emitter_state == 1)
	{
		telemeters.right_front.adc_value = ADC2ConvertedValues[0];
		telemeters.right_front.telemeter_value = telemeters.right_front.adc_value - telemeters.right_front.offset;
		HAL_GPIO_WritePin(GPIOB, TX_RIGHT_FRONT, RESET);
		telemeters.right_front.emitter_state = 0;
	}
	if (telemeters.left_front.emitter_state == 1)
	{
		telemeters.left_front.adc_value = ADC2ConvertedValues[1];
		telemeters.left_front.telemeter_value = telemeters.left_front.adc_value - telemeters.left_front.offset;
		HAL_GPIO_WritePin(GPIOB, TX_LEFT_FRONT, RESET);
		telemeters.left_front.emitter_state = 0;
	}
	if (telemeters.right_diag.emitter_state == 1)
	{
		telemeters.right_diag.adc_value = ADC2ConvertedValues[2];
		telemeters.right_diag.telemeter_value = telemeters.right_diag.adc_value - telemeters.right_diag.offset;
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.right_diag.emitter_state = 0;
	}
	if (telemeters.left_diag.emitter_state == 1)
	{
		telemeters.left_diag.adc_value = ADC2ConvertedValues[3];
		telemeters.left_diag.telemeter_value = telemeters.left_diag.adc_value - telemeters.left_diag.offset;
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.left_diag.emitter_state = 0;
	}

	telemeters.end_of_conversion++;
}

void Debug_Telemeter(void)
{
	while(1)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10, 0,  "LFRONT  = ", (int16_t) telemeters.left_front.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 9, "LDIAG   = ", (int16_t) telemeters.left_diag.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 18, "RDIAG   = ", (int16_t) telemeters.right_diag.telemeter_value, &Font_5x8);
		ssd1306PrintInt(10, 27, "RFRONT  = ", (int16_t) telemeters.right_front.telemeter_value, &Font_5x8);

		ssd1306PrintInt(10, 47, "interrupt cnt =  ", (int32_t) telemeters.it_cnt/10000, &Font_5x8);
		ssd1306PrintInt(10, 57, "end of conv.  =  ", (int32_t) telemeters.end_of_conversion/10000, &Font_5x8);
		ssd1306Refresh();
	}
}
