/**************************************************************************/
/*!
    @file    Encoders.c
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

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/encoders/ie512.h"

// Machine Definitions
typedef struct
{
	volatile double abs_dist;
	volatile double offset_dist;
	volatile double rel_dist;
	signed int mot_rev_cnt;
	TIM_HandleTypeDef *timer;
} encoder;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/**************************************************************************/
/* Structure init                                                 */
/**************************************************************************/
// Global variable

volatile encoder left_encoder =
{
		0,
		0,
		0,
		0,
		&htim1
};

volatile encoder right_encoder =
{
		0,
		0,
		0,
		0,
		&htim3
};

/* Static functions */
static int  	encoderResetDistance(encoder *enc);
static double 	encoderGetDistance(encoder *enc);

void encodersInit(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = ENCODER_RESOLUTION;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC2Filter = 0;
	HAL_TIM_Encoder_Init(&htim3, &sConfig);

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = ENCODER_RESOLUTION;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC2Filter = 0;
	HAL_TIM_Encoder_Init(&htim1, &sConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);

	left_encoder.mot_rev_cnt = 0;
	right_encoder.mot_rev_cnt = 0;
}

void encoderLeft_IT(void)
{
	switch (__HAL_TIM_DIRECTION_STATUS(&htim1))
	{
	case 1 :
		--left_encoder.mot_rev_cnt;
		break;
	case 0 :
		++left_encoder.mot_rev_cnt;
		break;
	}
}

void encoderRight_IT(void)
{
	switch (__HAL_TIM_DIRECTION_STATUS(&htim3))
	{
	case 1 :
		--right_encoder.mot_rev_cnt;
		break;
	case 0 :
		++right_encoder.mot_rev_cnt;
		break;
	}
}

/*  encoderResetDistance
 *  set offset to current absolute distance
 *  offset in millimeters
 */
int encoderResetDistance(encoder *enc)
{
	enc->offset_dist = 	((((double)enc->mot_rev_cnt * ENCODER_RESOLUTION) +
			((double)__HAL_TIM_GetCounter(enc->timer))) /
			STEPS_PER_MM);
	return IE512_DRIVER_E_SUCCESS;
}

/*  encoderGetDistance
 *  return current relative distance and set absolute distance
 *  distance in millimeters
 */
double encoderGetDistance(encoder *enc)
{
	enc->rel_dist = (((((double)enc->mot_rev_cnt * ENCODER_RESOLUTION) +
			((double)__HAL_TIM_GetCounter(enc->timer))) /
			STEPS_PER_MM) -
			(double)enc->offset_dist);
	enc->abs_dist = ((((double)enc->mot_rev_cnt * ENCODER_RESOLUTION) +
			((double)__HAL_TIM_GetCounter(enc->timer))) /
			STEPS_PER_MM);
	return enc->rel_dist;
}

int encodersReset(void)
{
    encoderResetDistance((encoder*)&left_encoder);
	encoderResetDistance((encoder*)&right_encoder);
	return IE512_DRIVER_E_SUCCESS;
}

double encoderGetDist(enum encoderName encoder_name)
{
	if (encoder_name == ENCODER_L)
		return encoderGetDistance((encoder*)&left_encoder);
	if (encoder_name == ENCODER_R)
		return encoderGetDistance((encoder*)&right_encoder);

	return IE512_DRIVER_E_ERROR;
}

// test encoder
void encoderTest(void)
{
	encodersInit();
	encodersReset();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AREA);

		ssd1306PrintInt(0, 5,  "L_DIST_REL =  ",(signed int) encoderGetDist(ENCODER_L), &Font_5x8);
		ssd1306PrintInt(0, 15, "L_DIST_ABS =  ",(signed int) left_encoder.abs_dist, &Font_5x8);

		ssd1306PrintInt(0, 25, "R_DIST_REL =  ",(signed int) encoderGetDist(ENCODER_R), &Font_5x8);
		ssd1306PrintInt(0, 35, "R_DIST_ABS =  ",(signed int) right_encoder.abs_dist, &Font_5x8);
		ssd1306DrawString(1, 53, "PRESS 'RIGHT' TO RESET REL. DIST.", &Font_3x6);
		if (expanderJoyFiltered() == JOY_RIGHT)
		{
			encodersReset();
		}
		ssd1306Refresh(MAIN_AREA);
		HAL_Delay(10);
	}
	antiBounceJoystick();
}
