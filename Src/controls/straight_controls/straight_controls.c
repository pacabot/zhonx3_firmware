/**************************************************************************/
/*!
    @file     Straight_Controls.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include <arm_math.h>

#include "stm32f4xx_hal.h"

#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"

#include "controls/straight_controls/straight_controls.h"
#include "controls/pid/pid.h"

/* extern variables ---------------------------------------------------------*/
/* global variables ---------------------------------------------------------*/
CONTROL_DEF trajectory_control;
arm_pid_instance_f32 gyro_pid_instance;
void Pwm_Init(void);

extern TIM_HandleTypeDef htim8;
TIM_OC_InitTypeDef sConfigOC;
GPIO_InitTypeDef GPIO_InitStruct;

//extern ADC_HandleTypeDef hadc1;
float i = 0;

extern volatile float gyro_Current_Angle;

int consigne = 0;
uint32_t Pulses[2] = {0,0};

void Straight_Control_Start(TypeOfSensors Sensor_x)
{
	consigne = 200;
	Pwm_Init();
	if(Sensor_x == GYRO)
	{
		gyro_pid_instance.Kp = 5;
		gyro_pid_instance.Ki = 0;//0.1;
		gyro_pid_instance.Kd = 0.4;
		trajectory_control.pid_instance = &gyro_pid_instance;
		Pid_Init(trajectory_control.pid_instance);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET); // start motors
	HAL_Delay(4000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET); // start motors
}

void Pwm_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 4;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1000;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim8);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim8);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCIDLESTATE_SET;
	htim8.Instance->CR2 &= ~TIM_CR2_CCPC;
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

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

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void Straight_Control_IT(void)
{
	trajectory_control.error_val = gyro_Current_Angle;
	trajectory_control.get_correction = Pid(trajectory_control.pid_instance, trajectory_control.error_val);

	Pulses[0] = consigne - trajectory_control.get_correction;
	Pulses[1] = consigne + trajectory_control.get_correction;

	sConfigOC.Pulse = (Pulses[0]);
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
	sConfigOC.Pulse = (Pulses[1]);
	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void Debug_Straight_Control(void)
{
	while(1)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "Correct =  ", (int32_t) gyro_Current_Angle, &Font_5x8);
		ssd1306PrintInt(10,  25, "Pulses[0] =  ", Pulses[0], &Font_5x8);
		ssd1306PrintInt(10,  35, "Pulses[1] =  ", Pulses[1], &Font_5x8);
		ssd1306Refresh();
	}
}
