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

#include "config/config.h"
#include "peripherals/display/ssd1306.h"
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
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);

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
}

/*## ADC Gyroscope callback for angle computing  #################################*/
	  /* -----------------------------------------------------------------------
	    Use TIM5 for start Injected conversion on ADC1 (gyro rate).
	      ----------------------------------------------------------------------- */
void adxrs620_ADC_IT(void)
{
	if (gyro.calibration_state == 0)
	{
		if (gyro.calibration_current_cycle < gyro.calibration_nb_cycle)
		{
			gyro.calibration_current_cycle ++;
			gyro.beta += (GYRO_COEFF*HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
		}
		else
		{
			gyro.calibration_state = 1;
			gyro.beta /= gyro.calibration_current_cycle;
		}
	}
	else if (gyro.calibration_state == 1)
	{
		gyro.current_angle += (GYRO_COEFF*HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)) - gyro.beta;  //optimized gyro integration DMA
		gyro.callback_cnt++;
	}
}

void adxrs620Test(void)
{
    adxrs620Init();
	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "Angle =  ", (int32_t) gyro.current_angle, &Font_5x8);
		ssd1306PrintInt(10,  15,  "cnt =  ", (int32_t) gyro.callback_cnt/1000, &Font_5x8);

		ssd1306PrintInt(10,  35,  "Beta =  ", (volatile double) (gyro.beta * 1000), &Font_5x8);
		ssd1306DrawString(80,  35,  ".10^-3", &Font_5x8);
		ssd1306Refresh();
	}
}

