/**************************************************************************/
/*!
    @file    Encoders.c
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
#include "peripherals/encoders/ie512.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/**************************************************************************/
/* Structure init                                                 */
/**************************************************************************/
// Global variable
ENCODER_DEF left_encoder         = {0, 0};   // left encoder structure
ENCODER_DEF right_encoder        = {0, 0};   // right encoder structure

void Encoders_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2047;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC1Filter = 3;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC2Filter = 3;
	HAL_TIM_Encoder_Init(&htim3, &sConfig);

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 2047;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC1Filter = 3;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV8;
	sConfig.IC2Filter = 3;
	HAL_TIM_Encoder_Init(&htim1, &sConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
}

void Left_Encoder_IT(void)
{
	switch (__HAL_TIM_DIRECTION_STATUS(&htim1))
	{
	case 1 :
		--left_encoder.nb_revolutions;
		break;
	case 0 :
		++left_encoder.nb_revolutions;
		break;
	}
}

void Right_Encoder_IT(void)
{
	switch (__HAL_TIM_DIRECTION_STATUS(&htim3))
	{
	case 1 :
		--right_encoder.nb_revolutions;
		break;
	case 0 :
		++right_encoder.nb_revolutions;
		break;
	}
}
// test encoder
void Debug_Encoder(void)
{

	while(1)
	{
		ssd1306ClearScreen();
		//		  ssd1306PrintInt(0, 6, "REV = ", toto, &Font_3x6);
		//		  ssd1306PrintInt(0, 13, "CNT = ",  (&htim1)->Instance->CNT, &Font_3x6);
		ssd1306PrintInt(0, 7, "L_REV =  ", left_encoder.nb_revolutions, &Font_3x6);
		ssd1306PrintInt(0, 14, "L_CNT =  ",  __HAL_TIM_GetCounter(&htim1), &Font_3x6);
		ssd1306PrintInt(0, 21, "L_DIR =  ",  __HAL_TIM_DIRECTION_STATUS(&htim1), &Font_3x6);

		ssd1306PrintInt(0, 35, "R_REV =  ", right_encoder.nb_revolutions, &Font_3x6);
		ssd1306PrintInt(0, 42, "R_CNT =  ",  __HAL_TIM_GetCounter(&htim3), &Font_3x6);
		ssd1306PrintInt(0, 49, "R_DIR =  ",  __HAL_TIM_DIRECTION_STATUS(&htim3), &Font_3x6);
		ssd1306Refresh();
		HAL_Delay(10);
	}
}


