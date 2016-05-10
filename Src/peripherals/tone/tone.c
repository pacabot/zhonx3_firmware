/**************************************************************************/
/*!
 @file    tone.c
 @author  BMO (PACABOT)
 @date    13/02/2015
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
#include "peripherals/telemeters/telemeters.h"

/* Middleware declarations */
#include "middleware/display/banner.h"

/* Declarations for this module */
#include "peripherals/tone/tone.h"

static void imperialMarch(void);
static void happyBirthday(void);

/* extern variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim11;
static int tone_duration = 0;

void toneInit(void)
{
    TIM_OC_InitTypeDef sConfigOC;

    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 1800;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 1000;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim11);

    HAL_TIM_PWM_Init(&htim11);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1);
}

void tonesplayer(int *note, int *duration, int size, int tempo)
{
    int note_freq;
    int uwPrescalerValue;
    for (int ii = 0; ii < size; ii++)
    {
        note_freq = note[ii];

        uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / (note_freq * 1000)) - 1;
        htim11.Instance = TIM11;
        htim11.Init.Prescaler = uwPrescalerValue;
        htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim11.Init.Period = 1000;
        htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        HAL_TIM_Base_Init(&htim11);
        HAL_TIM_PWM_Init(&htim11);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

        HAL_Delay(60000 * duration[ii] / (tempo * 4) - 60);
        HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
        HAL_Delay(60);
    }
}

// Arduino tone() compatible
void tone(int note, int duration)
{
    int uwPrescalerValue;
    tone_duration = 0;

    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / (note * 1000)) - 1;
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = uwPrescalerValue;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 1000 - 1;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim11);
    HAL_TIM_PWM_Init(&htim11);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    HAL_Delay(duration);
    HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

void toneItMode(int note, int duration_ms)
{
    int uwPrescalerValue;
    tone_duration = (duration_ms / (1000 / LOW_TIME_FREQ)) + 1;

    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / (note * 1000)) - 1;
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = uwPrescalerValue;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 1000 - 1;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim11);
    HAL_TIM_PWM_Init(&htim11);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

void tone_IT(void)
{
    if (tone_duration > 1)
        tone_duration--;
    if (tone_duration == 1)
        HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

void toneStart(int note)
{
    int uwPrescalerValue;
    tone_duration = 0;

    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / (note * 1000)) - 1;
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = uwPrescalerValue;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 1000 - 1;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim11);
    HAL_TIM_PWM_Init(&htim11);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

void toneStop(void)
{
    HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

void toneSetVolulme(int volume)
{
    TIM_OC_InitTypeDef sConfigOC;

    if (volume >= 100)
        volume = 100;

    if (volume <= 0)
        volume = 0;

    bannerSetIcon(BEEPER, volume);

    volume = volume * 20 / 100; //change scale

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = volume;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1);
}

void toneTest(void)
{
    toneSetVolulme(100);

    //	happyBirthday();
    imperialMarch();
}

void happyBirthday()
{
    tone(D2 * 2, 250 * 2);
    tone(D2 * 2, 250 * 2);
    tone(E2 * 2, 500 * 2);
    tone(D2 * 2, 500 * 2);
    tone(G2 * 2, 500 * 2);
    tone(F2H * 2, 1000 * 2);

    tone(D2 * 2, 250 * 2);
    tone(D2 * 2, 250 * 2);
    tone(E2 * 2, 500 * 2);
    tone(D2 * 2, 500 * 2);
    tone(A2 * 2, 500 * 2);
    tone(G2 * 2, 1000 * 2);

    tone(D2 * 2, 250 * 2);
    tone(D2 * 2, 250 * 2);
    tone(D3 * 2, 500 * 2);
    tone(B2 * 2, 500 * 2);
    tone(G2 * 2, 500 * 2);
    tone(F2H * 2, 500 * 2);
    tone(E2 * 2, 500 * 2);

    tone(C2 * 2, 250 * 2);
    tone(C2 * 2, 250 * 2);
    tone(B2 * 2, 500 * 2);
    tone(G2 * 2, 500 * 2);
    tone(A2 * 2, 500 * 2);
    tone(G2 * 2, 1000 * 2);

    HAL_Delay(1000);

    tone(C2 * 2, 200 * 2);
    tone(B2 * 2, 200 * 2);
    tone(A2 * 2, 1000 * 2);
    while (1)
    {
        tone(A2 * 2, 1 * 2);
        tone(A3 * 2, 1 * 2);
    }
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

