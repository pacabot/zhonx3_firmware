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

volatile double gyro_Current_Angle = 0.0;

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
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

	HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/*## ADC Gyroscope callback for angle computing  #################################*/
/* -----------------------------------------------------------------------
	    Use TIM5 for start Injected conversion on ADC1 (gyro rate).
	      ----------------------------------------------------------------------- */
void adxrs620_ADC_IT(void)
{
	gyro.current_angle += (GYRO_A_COEFF * (gyro.adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1))) - GYRO_B_COEFF;  //optimized gyro integration DMA

	gyro.callback_cnt++;
}

double adxrs620Calibrate(int nb_ech)
{
	double sample_sum = 0;
	for( int i = 0; i < nb_ech; i++)
	{
		sample_sum += GYRO_A_COEFF * gyro.adc_value;
		HAL_Delay(1);
	}
	return sample_sum / (double)nb_ech;
}

double GyroGetAngle(void)
{
	return gyro.current_angle;
}

void GyroResetAngle(void)
{
	gyro.current_angle = 0;
//	adxrs620Calibrate(10);
}

void adxrs620Test(void)
{
	adxrs620Init();
	GyroResetAngle();
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AERA);
		ssd1306PrintInt(10,  5,  "Angle =  ", (int32_t) GyroGetAngle(), &Font_5x8);
		ssd1306PrintInt(10,  15,  "cnt =  ", (int32_t) gyro.callback_cnt/1000, &Font_5x8);

		ssd1306PrintInt(10,  45,  "ADC val =  ", (int32_t) gyro.adc_value, &Font_5x8);

		ssd1306Refresh(MAIN_AERA);
	}
}

void adxrs620Cal(void)
{
	double cal;
	adxrs620Init();
	ssd1306ClearScreen(MAIN_AERA);
	ssd1306DrawString(0, 50, "DON'T TOUTCH Z3!!", &Font_3x6);
	ssd1306Refresh(MAIN_AERA);
	HAL_Delay(3000);
	cal = adxrs620Calibrate(5000);
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AERA);
		ssd1306PrintInt(10,  5,  "B =", (int32_t)(cal * 100000.00), &Font_5x8);
		ssd1306Refresh(MAIN_AERA);
	}
}

