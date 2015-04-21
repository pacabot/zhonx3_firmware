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
#include "peripherals/multimeter/multimeter.h"

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/gyroscope/adxrs620.h"

extern ADC_HandleTypeDef hadc1;

__IO uint32_t DMA_ADC_Gyro_Rate;

volatile float gyro_Current_Angle = 0.0;

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void adxrs620Init(void)
{
	ADC_InjectionConfTypeDef sConfigInjected;

	mulimeterInit();

	/**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

	adxrs620Calibrate(10);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void adxrs620Calibrate(int nb_ech)
{
	gyro.calibration_current_cycle = 0;
	gyro.calibration_nb_cycle = nb_ech;
	gyro.beta = 0;
	gyro.callback_cnt = 0;
	gyro.calibration_state = 0;
	gyro.max_adc_value = 0;
	gyro.min_adc_value = 0;
}

/*## ADC Gyroscope callback for angle computing  #################################*/
	  /* -----------------------------------------------------------------------
	    Use TIM5 for start Injected conversion on ADC1 (gyro rate).
	      ----------------------------------------------------------------------- */
void adxrs620_ADC_IT(void)
{
	uint32_t gyro_adc = 0;
	if (gyro.calibration_state == 0)
	{
		if (gyro.calibration_current_cycle < gyro.calibration_nb_cycle)
		{
			gyro.calibration_current_cycle ++;
			gyro.beta += (GYRO_COEFF*(gyro_adc += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)));
		}
		else
		{
			gyro.calibration_state = 1;
			gyro.beta /= gyro.calibration_current_cycle;
			gyro_adc /= gyro.calibration_current_cycle;
			gyro.max_adc_value = gyro_adc;
			gyro.min_adc_value = gyro_adc;
		}
	}
	else if (gyro.calibration_state == 1)
	{
		gyro.current_angle += (GYRO_COEFF*(gyro_adc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1))) - gyro.beta;  //optimized gyro integration DMA
		if (gyro_adc > gyro.max_adc_value)
			gyro.max_adc_value = gyro_adc;
		if (gyro_adc < gyro.min_adc_value)
			gyro.min_adc_value = gyro_adc;
		gyro.adc_value = gyro_adc;
		gyro.callback_cnt++;
	}
}

void adxrs620Test(void)
{
    adxrs620Init();
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "Angle =  ", (int32_t) gyro.current_angle, &Font_5x8);
		ssd1306PrintInt(10,  15,  "cnt =  ", (int32_t) gyro.callback_cnt/1000, &Font_5x8);

		ssd1306PrintInt(10,  25,  "max ADC =  ", (int32_t) gyro.max_adc_value, &Font_5x8);
		ssd1306PrintInt(10,  35,  "min ADC =  ", (int32_t) gyro.min_adc_value, &Font_5x8);
		ssd1306PrintInt(10,  45,  "ADC val =  ", (int32_t) gyro.adc_value, &Font_5x8);

		ssd1306PrintInt(10,  55,  "Beta =  ", (volatile double) (gyro.beta * 1000), &Font_5x8);
		ssd1306DrawString(80,  55,  ".10^-3", &Font_5x8);
		ssd1306Refresh();
	}
}

