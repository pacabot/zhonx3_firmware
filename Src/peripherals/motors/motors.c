/**************************************************************************/
/*!
    @file    motors.c
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
#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/motors/motors.h"

motor left_motor =
{
	LEFT_MOTOR_IN1,
	LEFT_MOTOR_IN2
};

motor right_motor =
{
	RIGHT_MOTOR_IN1,
	RIGHT_MOTOR_IN2
};

extern TIM_HandleTypeDef MOTORS_TIMER;

void Motors_Pwm_Init(void)
{
	TIM_OC_InitTypeDef sConfigOC;
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 40;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 99;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim8);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim8);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCIDLESTATE_SET;
	htim8.Instance->CR2 &= ~TIM_CR2_CCPC;
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);
}

//void Motors_Decay_Mode()
//{
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
//
//	HAL_TIM_PWM_Start(&htim8,  R_MOTOR_TIN2);
//	HAL_TIM_PWM_Start(&htim8,  L_MOTOR_TIN1);
//}

void Motors_Start()
{
	HAL_GPIO_WritePin(GPIOA, MOTORS_STANDBY, SET);
}

void Motors_Stop()
{
	HAL_GPIO_WritePin(GPIOA, MOTORS_STANDBY, RESET);
}

void setMotor(motor *mot, int isForward, int duty, int isSlowDecay)
{
	TIM_OC_InitTypeDef sConfigOC;

	// Limit duty cycle to its maximum value
	if (duty > 99)
	{
		duty = 99;
	}

	// Reverse left motor
	if (mot == &left_motor)
	{
		isForward = reverse_bit(isForward);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.OCNPolarity = TIM_OCNIDLESTATE_RESET;

	if (duty == 0)
	{
		// Brake
		HAL_TIM_PWM_Stop(&MOTORS_TIMER, mot->IN1);
		HAL_TIM_PWM_Stop(&MOTORS_TIMER, mot->IN2);
		return;
	}

	if (isSlowDecay == 1)
	{
		sConfigOC.Pulse = 99 - duty;
	}
	else
	{
		sConfigOC.Pulse = duty;
	}

	/* if:
	 * 	- Forward Fast Decay
	 * 	or
	 * 	- Backward Fast Decay
	 */
	if ((isForward == 1) && (isSlowDecay == 0))
	{
		// Send PWM on IN1
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN1);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN1);

		//Set IN2 to 0
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN2);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN2);
	}
	else if ((isForward == 0) && (isSlowDecay == 0))
	{
		// Send PWM on IN2
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN2);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN2);

		// Set IN1 to 0
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN1);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN1);
	}

	/* if:
	 * 	- Forward Slow Decay
	 * 	or
	 * 	- Backward Slow Decay
	 */
	if ((isForward == 1) && (isSlowDecay == 1))
	{
		// Send PWM on IN2
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN2);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN2);

		// Set IN1 to 1
		sConfigOC.Pulse = 99;
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN1);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN1);
	}
	else if ((isForward == 0) && (isSlowDecay == 1))
	{
		// Send PWM on IN1
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN1);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN1);

		// Set IN2 to 1
		sConfigOC.Pulse = 99;
		HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN2);
		HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN2);
	}
}

void freeMotor(motor *mot)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.OCNPolarity = TIM_OCNIDLESTATE_RESET;
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN1);
	HAL_TIM_PWM_ConfigChannel(&MOTORS_TIMER, &sConfigOC, mot->IN2);
	HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN1);
	HAL_TIM_PWM_Start(&MOTORS_TIMER, mot->IN2);
}

void motors_test(void)
{
	volatile long leftCounter = 0;
	volatile long rightCounter = 0;
	volatile int direction = 0;
	volatile int cnt = 0;
	volatile int sum = 0;

	// Forward Fast (PWM on IN1, LOW on IN2)
//	setMotor(&left_motor, 1, 20, 0);
	cnt = 0;
	sum = 0;
	leftCounter = 0;

	int i = 0;
	int decay = DECAY_SLOW;
	int result = 0;

	result = sum / (cnt * 2048);

	//	  freeMotor(&left_motor);
	//	  //setMotor(&left_motor, 1, 0, 0);
	//	  HAL_Delay(1000);
	//
	//	  // Reverse Fast (LOW on IN1, PWM on IN2)
	//	  setMotor(&left_motor, 0, 20, 0);
	//	  HAL_Delay(2000);
	//	  freeMotor(&left_motor);
	////	  setMotor(&left_motor, 0, 0, 0);
	//	  HAL_Delay(1000);
	//
	//	  // Forward Slow (HIGH on IN1, PWM on IN2)
	//	  setMotor(&left_motor, 1, 20, 1);
	//	  HAL_Delay(2000);
	//	  freeMotor(&left_motor);
	////	  setMotor(&left_motor, 1, 0, 1);
	//	  HAL_Delay(1000);
	//
	//	  // Reverse Slow (PWM on IN1, HIGH on IN2)
	//	  setMotor(&left_motor, 0, 20, 1);
	//	  HAL_Delay(2000);
	//	  freeMotor(&left_motor);
	////	  setMotor(&left_motor, 0, 0, 1);
	//	  HAL_Delay(1000);
	//

//	Motors_Start();
	for (i = 0; i < 20; i += 1)
	{
		setMotor(&left_motor, DIRECTION_FORWARD, i, decay);
		Motors_Start();
		setMotor(&right_motor, DIRECTION_FORWARD, i, decay);
		HAL_Delay(50);
	}

	HAL_Delay(1000);

	for (i = 20; i > 0; i -= 1)
	{
		setMotor(&left_motor, DIRECTION_FORWARD, i, decay);
		setMotor(&right_motor, DIRECTION_FORWARD, i, decay);
		HAL_Delay(50);
	}

	setMotor(&left_motor, DIRECTION_FORWARD, 0, decay);
	HAL_Delay(1000);

	for (i = 0; i < 20; i += 1)
	{
		setMotor(&left_motor, DIRECTION_BACKWARD, i, decay);
		setMotor(&right_motor, DIRECTION_BACKWARD, i, decay);
		HAL_Delay(50);
	}

	HAL_Delay(1000);

	for (i = 20; i > 0; i -= 1)
	{
		setMotor(&left_motor, DIRECTION_BACKWARD, i, decay);
		setMotor(&right_motor, DIRECTION_BACKWARD, i, decay);
		HAL_Delay(50);
	}

	setMotor(&left_motor, DIRECTION_BACKWARD, 0, decay);
	setMotor(&right_motor, DIRECTION_BACKWARD, 0, decay);
}

void Debug_Motors(void)
{
	while(1)
	{
		ssd1306ClearScreen();
		//		ssd1306PrintInt(10,  5,  "Angle =  ", (int32_t) gyro.current_angle, &Font_5x8);
		//		ssd1306PrintInt(10,  15,  "cnt =  ", (int32_t) gyro.callback_cnt/10000, &Font_5x8);
		//
		//		ssd1306PrintInt(10,  35,  "Beta =  ", (volatile double) (gyro.beta * 1000), &Font_5x8);
		//		ssd1306DrawString(80,  35,  ".10^-3", &Font_5x8);
		ssd1306Refresh();
	}
}
