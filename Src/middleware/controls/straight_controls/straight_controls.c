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

#include "middleware/controls/straight_controls/straight_controls.h"
#include "middleware/controls/pidController/pidController.h"

#include "peripherals/telemeters/telemeters.h"
#include "peripherals/motors/motors.h"

/* extern variables ---------------------------------------------------------*/
/* global variables ---------------------------------------------------------*/
CONTROL_DEF trajectory_control;
arm_pid_instance_f32 gyro_pid_instance;

GPIO_InitTypeDef GPIO_InitStruct;

//extern ADC_HandleTypeDef hadc1;
float i = 0;

extern volatile float gyro_Current_Angle;

int consigne = 0;
int Pulses[2] = {0,0};

void straightControlStart(TypeOfSensors Sensor_x)
{
	consigne = 200;
	if(Sensor_x == GYRO)
	{
		gyro_pid_instance.Kp = 5;
		gyro_pid_instance.Ki = 0;//0.1;
		gyro_pid_instance.Kd = 0.4;
		trajectory_control.pid_instance = &gyro_pid_instance;
		pidControllerInit(trajectory_control.pid_instance);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET); // start motors
	HAL_Delay(4000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET); // start motors
}

void straightControl_IT(void)
{
	static int consigne = 100;
	trajectory_control.error_val = telemeters.left_diag.telemeter_value - telemeters.right_diag.telemeter_value;
	trajectory_control.get_correction = pidController(trajectory_control.pid_instance, trajectory_control.error_val);

	Pulses[0] = consigne - trajectory_control.get_correction;
	Pulses[1] = consigne + trajectory_control.get_correction;

	motorSet(&left_motor, DIRECTION_FORWARD, Pulses[0], DECAY_FAST);
	motorSet(&right_motor, DIRECTION_FORWARD, Pulses[1], DECAY_FAST);
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
