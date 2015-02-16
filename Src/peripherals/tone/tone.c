/**************************************************************************/
/*!
    @file    tone.c
    @author  BMO (PACABOT)
    @date    13/02/2015
    @version  0.0
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "middleware/util/itoa.h"
#include "peripherals/tone/tone.h"

void imperialMarch(void);

extern TIM_HandleTypeDef htim11;

void tonesplayer(int *note, int *duration, int size, int tempo)
{
	int FREQ_NOTE=240;
	int uwPrescalerValue=1800;
	for (int ii=0;ii<size;ii++)
	{
		FREQ_NOTE = note[ii];

		uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (FREQ_NOTE * 1000)) - 1;
		htim11.Instance = TIM11;
		htim11.Init.Prescaler = uwPrescalerValue;
		htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim11.Init.Period = 1000;
		htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		HAL_TIM_Base_Init(&htim11);
		HAL_TIM_PWM_Init(&htim11);
		HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

		HAL_Delay(60000*duration[ii] / (tempo*4) - 60);
		HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
		HAL_Delay(60);
	}
}

// Arduino tone() compatible
void tone(int note, int duration)
{
	int uwPrescalerValue = 1800;

	uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (note * 1000)) - 1;
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = uwPrescalerValue;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 1000-1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim11);
	HAL_TIM_PWM_Init(&htim11);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

	HAL_Delay(duration);
	HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

void toneSetVolulme(int volume)
{
	TIM_OC_InitTypeDef sConfigOC;

	if (volume >= 100)
		volume = 100;

	if (volume <= 0)
		volume = 1;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = volume;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1);
}

void toneTest(void)
{
	toneSetVolulme(2);
	imperialMarch();
}

void imperialMarch()
{

	//Play first section
	tone(a, 500);
	tone(a, 500);
	tone(a, 500);
	tone(f, 350);
	tone(cH, 150);
	tone(a, 500);
	tone(f, 350);
	tone(cH, 150);
	tone(a, 650);

	HAL_Delay(500);

	tone(eH, 500);
	tone(eH, 500);
	tone(eH, 500);
	tone(fH, 350);
	tone(cH, 150);
	tone(gS, 500);
	tone(f, 350);
	tone(cH, 150);
	tone(a, 650);

	HAL_Delay(500);

	//Play second section
	tone(aH, 500);
	tone(a, 300);
	tone(a, 150);
	tone(aH, 500);
	tone(gSH, 325);
	tone(gH, 175);
	tone(fSH, 125);
	tone(fH, 125);
	tone(fSH, 250);

	HAL_Delay(325);

	tone(aS, 250);
	tone(dSH, 500);
	tone(dH, 325);
	tone(cSH, 175);
	tone(cH, 125);
	tone(b, 125);
	tone(cH, 250);

	HAL_Delay(350);

	//Variant 1
	tone(f, 250);
	tone(gS, 500);
	tone(f, 350);
	tone(a, 125);
	tone(cH, 500);
	tone(a, 375);
	tone(cH, 125);
	tone(eH, 650);

	HAL_Delay(500);

	//Repeat second section
	tone(aH, 500);
	tone(a, 300);
	tone(a, 150);
	tone(aH, 500);
	tone(gSH, 325);
	tone(gH, 175);
	tone(fSH, 125);
	tone(fH, 125);
	tone(fSH, 250);

	HAL_Delay(325);

	tone(aS, 250);
	tone(dSH, 500);
	tone(dH, 325);
	tone(cSH, 175);
	tone(cH, 125);
	tone(b, 125);
	tone(cH, 250);

	HAL_Delay(350);

	//Variant 2
	tone(f, 250);
	tone(gS, 500);
	tone(f, 375);
	tone(cH, 125);
	tone(a, 500);
	tone(f, 375);
	tone(cH, 125);
	tone(a, 650);

	HAL_Delay(650);
}

