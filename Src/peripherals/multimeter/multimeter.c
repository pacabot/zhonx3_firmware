/**************************************************************************/
/*!
    @file     multimeter.c
    @author   PLF Pacabot.com
    @date     01 January 2015
    @version  0.10
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "config/config.h"
#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/multimeter/multimeter.h"

extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

//volatile double gyro  = (ADC_VOLTAGE/(4095*GYRO_TEMP_COEFF));
volatile double betaMulti = 0;

__IO uint16_t ADC1MultimeterConvertedValues[10] = {0};

/**************************************************************************/
/*!
    RANK 1		CHANNEL 7						GYRO_TEMP
    RANK 2		CHANNAL TEMPERATURE SENSOR		TEMPERATURE_SENSOR
    RANK 3		CHANNEL 15						VBAT
    RANK 4		CHANNEL INTERNAL VBAT			INTERNAL VBAT
 */
/**************************************************************************/
void Init_Mulimeter(void)
{
	multimeter.timer_cnt = 0;
	multimeter.get_vbat_state = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1MultimeterConvertedValues,4);
}

void Multimeter_IT(void)
{
	multimeter.timer_cnt++;
	multimeter.get_vbat_state = 1;
	HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, SET);
}

void Multimeter_REGULAR_ADC_IT(void)
{
			HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, RESET);
			multimeter.get_vbat_state++;

			multimeter.gyro_temp.value = (GYRO_T_COEFF_A * ADC1MultimeterConvertedValues[0]) + GYRO_T_COEFF_B;
			multimeter.stm32_temp.value = (STM32_T_COEFF_A * ADC1MultimeterConvertedValues[1]) + STM32_T_COEFF_B;
			multimeter.vbat.value = (ADC1MultimeterConvertedValues[2])*VBAT_BRIDGE_COEFF;
}

void Debug_Mulimeter(void)
{
	while(1)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "timer count =  ", (int32_t) multimeter.timer_cnt, &Font_5x8);
		ssd1306PrintInt(10,  15,  "get vbat =  ", (int32_t) multimeter.get_vbat_state, &Font_5x8);
		ssd1306PrintInt(10,  35,  "Temp. Gyro =  ", multimeter.gyro_temp.value, &Font_5x8);
		ssd1306PrintInt(10,  45,  "Temp. STM32 =  ", multimeter.stm32_temp.value, &Font_5x8);
		ssd1306PrintInt(10,  55,  "ADC1 3 =  ", multimeter.vbat.value, &Font_5x8);
		ssd1306Refresh();
	}
}
