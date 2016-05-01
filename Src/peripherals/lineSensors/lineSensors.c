/**************************************************************************/
/*!
 @file    lineSensor.c
 @author   PLF Pacabot.com
 @date     01 December 2014
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
#include "middleware/display/banner.h"

/* Declarations for this module */
#include "peripherals/lineSensors/lineSensors.h"

/* Definition for ADCx GPIO Pin */
#define TX_LINESENSORS              GPIO_PIN_2      //PA2

/* Definition for ADCx's Channel */
#define RX_LEFT_EXT                 ADC_CHANNEL_3   //ADC3
#define RX_LEFT                     ADC_CHANNEL_4   //ADC2
#define RX_FRONT                    ADC_CHANNEL_1   //ADC3
#define RX_RIGHT                    ADC_CHANNEL_13  //ADC2
#define RX_RIGHT_EXT                ADC_CHANNEL_12  //ADC3

/* Types definitions */
typedef struct
{
    uint32_t adc_value;
    uint32_t ref_adc_value;
} lineSensors_state;

typedef struct
{
    lineSensors_state left_ext;
    lineSensors_state left;
    lineSensors_state front;
    lineSensors_state right;
    lineSensors_state right_ext;
    char active_ADC2;
    char active_ADC3;
    char emitter_state;
    char active_state;
    char selector;
} lineSensors_struct;

volatile lineSensors_struct lineSensors;

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

ADC_InjectionConfTypeDef sConfigInjected;

GPIO_InitTypeDef GPIO_InitStruct;
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */

/**************************************************************************/
/*!
 RANK 1		ADC3  	ADC 3 				RX_LEFT_EXT
 RANK 2		ADC4  	ADC 2 				RX_LEFT
 RANK 2		ADC1  	ADC 3 				RX_FRONT
 RANK 1		ADC13 	ADC 2 				RX_RIGHT
 RANK 3		ADC12 	ADC 3 				RX_RIGHT_EXT
 */
/**************************************************************************/
void lineSensorsInit(void)
{
    HAL_ADC_Stop_IT(&hadc2);
    HAL_ADC_Stop_IT(&hadc3);
    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = RX_LEFT;
    sConfigInjected.InjectedRank = 2;
    sConfigInjected.InjectedNbrOfConversion = 2;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = RX_RIGHT;
    sConfigInjected.InjectedRank = 1;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = RX_LEFT_EXT;
    sConfigInjected.InjectedRank = 1;
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = RX_FRONT;
    sConfigInjected.InjectedRank = 2;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = RX_RIGHT_EXT;
    sConfigInjected.InjectedRank = 3;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    hadc3.Init.Resolution = ADC_RESOLUTION12b;
    hadc3.Init.ScanConvMode = ENABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.NbrOfDiscConversion = 0;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 0;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc3);

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    hadc2.Init.Resolution = ADC_RESOLUTION12b;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 0;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc2);

    HAL_ADC_Stop_IT(&hadc3);
    HAL_ADC_Stop_IT(&hadc2);

    memset((lineSensors_struct*) &lineSensors, 0, sizeof(lineSensors_struct));
}

void lineSensorsStart(void)
{
    lineSensors.active_state = TRUE;
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    bannerSetIcon(LINESENSORS, TRUE);
}

void lineSensorsStop(void)
{
    lineSensors.active_state = FALSE;
    HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
    HAL_ADCEx_InjectedStop_IT(&hadc2);
    HAL_ADCEx_InjectedStop_IT(&hadc3);
    bannerSetIcon(LINESENSORS, FALSE);
}

double getLineSensorAdc(enum linesensorName linesensor_name)
{
    switch (linesensor_name)
    {
        case LINESENSOR_EXT_L:
            return lineSensors.left_ext.adc_value;
        case LINESENSOR_L:
            return lineSensors.left.adc_value;
        case LINESENSOR_F:
            return lineSensors.front.adc_value;
        case LINESENSOR_R:
            return lineSensors.right.adc_value;
        case LINESENSOR_EXT_R:
            return lineSensors.right_ext.adc_value;
        default :
            return 0.00;
    }
}

void lineSensors_IT(void)
{
    if (lineSensors.active_state == FALSE)
        return;

    lineSensors.selector++;

    if (lineSensors.selector > 1)
    {
        lineSensors.selector = 0;
    }

    switch (lineSensors.selector)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, SET);
            lineSensors.emitter_state = TRUE;
            break;
        case 1:
            lineSensors.active_ADC2 = TRUE;
            lineSensors.active_ADC3 = TRUE;
            HAL_ADCEx_InjectedStart_IT(&hadc2);
            HAL_ADCEx_InjectedStart_IT(&hadc3);
            break;
//        case 2:
//            HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
//            lineSensors.emitter_state = FALSE;
//            break;
//        case 3:
//            lineSensors.active_ADC2 = TRUE;
//            lineSensors.active_ADC3 = TRUE;
//            HAL_ADCEx_InjectedStart_IT(&hadc2);
//            HAL_ADCEx_InjectedStart_IT(&hadc3);
//            break;
    }
}

void lineSensors_ADC_IT(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2)
    {
        lineSensors.right.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        lineSensors.left.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        lineSensors.active_ADC2 = FALSE;
    }
    if (hadc == &hadc3)
    {
        lineSensors.left_ext.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
        lineSensors.front.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
        lineSensors.right_ext.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);
        lineSensors.active_ADC3 = FALSE;
    }
    if (lineSensors.active_ADC2 == FALSE && lineSensors.active_ADC3 == FALSE)
    {
        HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
        lineSensors.emitter_state = FALSE;
    }

//    if (lineSensors.emitter_state == TRUE)
//    {
//        if (hadc == &hadc2)
//        {
//            if (HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) > lineSensors.right.ref_adc_value)
//                lineSensors.right.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - lineSensors.right.ref_adc_value;
//            else
//                lineSensors.right.adc_value = 0;
//            if (HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) > lineSensors.left.ref_adc_value)
//                lineSensors.left.adc_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) - lineSensors.left.ref_adc_value;
//            else
//                lineSensors.left.adc_value = 0;
//            lineSensors.active_ADC2 = FALSE;
//        }
//        if (hadc == &hadc3)
//        {
//            if (HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) > lineSensors.left_ext.ref_adc_value)
//                lineSensors.left_ext.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - lineSensors.left_ext.ref_adc_value;
//            else
//                lineSensors.left_ext.adc_value = 0;
//            if (HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2) > lineSensors.front.ref_adc_value)
//                lineSensors.front.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2) - lineSensors.front.ref_adc_value;
//            else
//                lineSensors.front.adc_value = 0;
//            if (HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) > lineSensors.right_ext.ref_adc_value)
//                lineSensors.right_ext.adc_value = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) - lineSensors.right_ext.ref_adc_value;
//            else
//                lineSensors.right_ext.adc_value = 0;
//            lineSensors.active_ADC3 = FALSE;
//        }
//        if (lineSensors.active_ADC2 == FALSE && lineSensors.active_ADC3 == FALSE)
//        {
//            HAL_GPIO_WritePin(GPIOA, TX_LINESENSORS, RESET);
//            lineSensors.emitter_state = FALSE;
//        }
//    }
//    else
//    {
//        if (hadc == &hadc2)
//        {
//            lineSensors.right.ref_adc_value = 4095 - HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//            lineSensors.left.ref_adc_value = 4095 - HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
//            lineSensors.active_ADC2 = FALSE;
//        }
//        if (hadc == &hadc3)
//        {
//            lineSensors.left_ext.ref_adc_value = 4095 - HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
//            lineSensors.front.ref_adc_value = 4095 - HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
//            lineSensors.right_ext.ref_adc_value = 4095 - HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);
//            lineSensors.active_ADC3 = FALSE;
//        }
//    }
}

void lineSensorsTest(void)
{

    lineSensorsInit();
    lineSensorsStart();

    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(0, 0, &Font_5x8, "L_EXT = %d  %d", (uint16_t)getLineSensorAdc(LINESENSOR_EXT_L), lineSensors.left_ext.ref_adc_value);
        ssd1306PrintfAtLine(0, 1, &Font_5x8, "L     = %d  %d", (uint16_t)getLineSensorAdc(LINESENSOR_L), lineSensors.left.ref_adc_value);
        ssd1306PrintfAtLine(0, 2, &Font_5x8, "F     = %d  %d", (uint16_t)getLineSensorAdc(LINESENSOR_F), lineSensors.front.ref_adc_value);
        ssd1306PrintfAtLine(0, 3, &Font_5x8, "R     = %d  %d", (uint16_t)getLineSensorAdc(LINESENSOR_R), lineSensors.right.ref_adc_value);
        ssd1306PrintfAtLine(0, 4, &Font_5x8, "R_EXT = %d  %d", (uint16_t)getLineSensorAdc(LINESENSOR_EXT_R), lineSensors.right_ext.ref_adc_value);
        ssd1306Refresh();
    }
    lineSensorsStop();
}

