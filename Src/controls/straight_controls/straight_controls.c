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

GPIO_InitTypeDef GPIO_InitStruct;

//extern ADC_HandleTypeDef hadc1;
float i = 0;

extern volatile float gyro_Current_Angle;

int consigne = 0;
uint32_t Pulses[2] = {0,0};

void straightControlStart(TypeOfSensors Sensor_x)
{
	consigne = 200;
	if(Sensor_x == GYRO)
	{
		gyro_pid_instance.Kp = 5;
		gyro_pid_instance.Ki = 0;//0.1;
		gyro_pid_instance.Kd = 0.4;
		trajectory_control.pid_instance = &gyro_pid_instance;
		pidInit(trajectory_control.pid_instance);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET); // start motors
	HAL_Delay(4000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET); // start motors
}

void straightControl_IT(void)
{
//	trajectory_control.error_val = gyro_Current_Angle;
//	trajectory_control.get_correction = Pid(trajectory_control.pid_instance, trajectory_control.error_val);
//
//	Pulses[0] = consigne - trajectory_control.get_correction;
//	Pulses[1] = consigne + trajectory_control.get_correction;
//
//	sConfigOC.Pulse = (Pulses[0]);
//	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
//	sConfigOC.Pulse = (Pulses[1]);
//	HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
//
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void straightControlTest(void)
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
