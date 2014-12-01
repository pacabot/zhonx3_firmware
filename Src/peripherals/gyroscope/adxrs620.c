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

#include "peripherals/display/ssd1306.h"

#include "peripherals/gyroscope/adxrs620.h"

volatile double Alfa = (GYRO_VOLTAGE/(4095*GYRO_SENSITIVITY*HI_TIME_FREQ));
volatile double Beta = 0;

extern ADC_HandleTypeDef hadc1;

__IO uint32_t DMA_ADC_Gyro_Rate;

volatile float gyro_Current_Angle = 0.0;

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void ADXRS620_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	HAL_ADC_Stop_DMA(&hadc1);
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&DMA_ADC_Gyro_Rate, 1);
	HAL_ADC_Start(&hadc1);

	HAL_Delay(300);
	ADXRS620_Calibrate(50);
}

void ADXRS620_Calibrate(int nb_ech)
{
	for (int i = 0; i < nb_ech; i++)
	{
		Beta += (Alfa*(int32_t) DMA_ADC_Gyro_Rate);
		HAL_Delay(1);
	}

	Beta /= nb_ech;
}

void ADXRS620_IT(void)
{
	gyro_Current_Angle += (Alfa*(int32_t) DMA_ADC_Gyro_Rate) - Beta;  //optimized gyro integration DMA
}

void Debug_ADXRS620(void)
{
	while(1)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "Angle =  ", (int32_t) gyro_Current_Angle, &Font_5x8);
		ssd1306PrintInt(10,  15,  "Alfa =  ", (Alfa * 10000), &Font_5x8);
		ssd1306PrintInt(10,  25,  "Beta =  ", (int32_t) (Beta * 100), &Font_5x8);
		ssd1306Refresh();
	}
}

