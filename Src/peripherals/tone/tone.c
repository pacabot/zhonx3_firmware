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

#include "util/itoa.h"
#include "peripherals/tone/tone.h"

extern TIM_HandleTypeDef htim4;

void player_music(int *Note, int *Duree, int size, int tempo)
{

	int FREQ_NOTE=240;
	int uwPrescalerValue=1800;
	for (int ii=0;ii<size;ii++)
	{
		FREQ_NOTE = Note[ii];

		htim4.Instance = TIM4;

		uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (FREQ_NOTE * 100)) - 1;
		htim4.Init.Prescaler = uwPrescalerValue;
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = 50;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		HAL_TIM_PWM_Init(&htim4);
		if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
		{
			/* Starting Error */
			//    Error_Handler();
		}
		//1000 = 60 pulse /mn
		//  HAL_Delay(60000 / (tempo*Duree[ii]));
		HAL_Delay(60000*Duree[ii] / (tempo*4) - 60);
		if(HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) != HAL_OK)
		{
			/* Starting Error */
			//    Error_Handler();
		}
		HAL_Delay(60);
	}
}

