/**************************************************************************/
/*!
    @file     multimeter.c
    @author   PLF Pacabot.com
    @date     01 January 2015
    @version  0.10
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

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
#include "peripherals/multimeter/multimeter.h"

volatile multimeter_struct multimeter;

extern ADC_HandleTypeDef hadc1;

__IO uint16_t ADC1MultimeterConvertedValues[10] = {0};

/**************************************************************************/
/*!
    RANK 1		CHANNEL 7						GYRO_TEMP
    RANK 2		CHANNAL TEMPERATURE SENSOR		TEMPERATURE_SENSOR
    RANK 3		CHANNEL 15						VBAT
    RANK 4		CHANNEL INTERNAL VBAT			INTERNAL VBAT
 */
/**************************************************************************/
void mulimeterInit(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = EOC_SEQ_CONV;
	HAL_ADC_Init(&hadc1);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 2;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */

	multimeter.timer_cnt = 0;
	multimeter.get_vbat_state = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1MultimeterConvertedValues,3);
}

void multimeter_IT(void)
{
	multimeter.timer_cnt++;
	multimeter.get_vbat_state = 1;
	HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, SET);
}

void multimeter_ADC_IT(void)
{
	HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, RESET);
	multimeter.get_vbat_state++;

	multimeter.gyro_temp.value = (GYRO_T_COEFF_A * ADC1MultimeterConvertedValues[0]) + GYRO_T_COEFF_B;
	multimeter.stm32_temp.value = (STM32_T_COEFF_A * ADC1MultimeterConvertedValues[1]) + STM32_T_COEFF_B;
	multimeter.vbat.value = (ADC1MultimeterConvertedValues[2])*VBAT_BRIDGE_COEFF;
}

void mulimeterTest(void)
{

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "timer count =  ", (int32_t) multimeter.timer_cnt, &Font_5x8);
		ssd1306PrintInt(10,  15,  "get vbat =  ", (int32_t) multimeter.get_vbat_state, &Font_5x8);
		ssd1306PrintInt(10,  35,  "Temp. Gyro =  ", multimeter.gyro_temp.value, &Font_5x8);
		ssd1306PrintInt(10,  45,  "Temp. STM32 =  ", multimeter.stm32_temp.value, &Font_5x8);
		ssd1306PrintInt(10,  55,  "vbat (mV) =  ", multimeter.vbat.value, &Font_5x8);
		ssd1306Refresh();
	}
	antiBounceJoystick();
}
