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
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/positionControl.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */
#include "peripherals/multimeter/multimeter.h"

/* Definition for ADCx Channel Pin */
#define GET_ADC_BAT			GPIO_PIN_0

/* Types definitions */
typedef struct
{
    float value;
    uint16_t offset;
    uint16_t higher_interval;
    uint16_t lower_interval;
    char under_value;
    char over_value;
} channel_meter;

typedef struct
{
    channel_meter vbat;
    channel_meter gyro_temp;
    channel_meter stm32_temp;
    uint32_t timer_cnt;
    char active_state;
    char gpio_vbat_state;
} multimeter_struct;

volatile multimeter_struct multimeter;

extern ADC_HandleTypeDef hadc1;

__IO uint16_t ADC1MultimeterConvertedValues[10] = { 0 };

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
    hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION12b;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
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
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
    multimeter.gpio_vbat_state = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1MultimeterConvertedValues, 3);
}

void multimeter_IT(void)
{
    multimeter.gpio_vbat_state = TRUE;
    HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, SET);
    multimeter.timer_cnt++;
}

void multimeter_ADC_IT(void)
{
    multimeter.gyro_temp.value = (GYRO_T_COEFF_A * (float) ADC1MultimeterConvertedValues[0]) + GYRO_T_COEFF_B;
    multimeter.stm32_temp.value = (STM32_T_COEFF_A * (float) ADC1MultimeterConvertedValues[1]) + STM32_T_COEFF_B;
    if (multimeter.gpio_vbat_state == TRUE)
        multimeter.vbat.value = ((float) ADC1MultimeterConvertedValues[2]) * VBAT_BRIDGE_COEFF;
    multimeter.gpio_vbat_state = FALSE;
    HAL_GPIO_WritePin(GPIOB, GET_ADC_BAT, RESET);
}

float multimeterGetBatVoltage(void)
{
    return multimeter.vbat.value;
}

float multimeterSTM32Temp(void)
{
    return multimeter.stm32_temp.value;
}

float multimeterGyroTemp(void)
{
    return multimeter.gyro_temp.value;
}

void mulimeterTest(void)
{
    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintIntAtLine(0, 0, "Temp. Gyro =  ", (int) (multimeterGyroTemp() * 10.00), &Font_5x8);
        ssd1306PrintIntAtLine(0, 1, "Temp. STM32 =  ", (int) (multimeterSTM32Temp() * 10.00), &Font_5x8);
        ssd1306PrintIntAtLine(0, 3, "vbat (mV) =  ", (int) multimeterGetBatVoltage(), &Font_5x8);
        ssd1306Refresh();
    }
}
