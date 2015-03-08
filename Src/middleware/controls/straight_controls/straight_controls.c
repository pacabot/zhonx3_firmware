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

/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include "stdbool.h"

#include <arm_math.h>
#include <stdio.h>
#include <math.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"


/* Middleware declarations */
#include <middleware/controls/pidController/pidController.h>
#include <middleware/controls/motionControl/speedControl.h>

/* Declarations for this module */
#include "middleware/controls/straight_controls/straight_controls.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */

/* extern variables ---------------------------------------------------------*/
struct control control = {0};

/* global variables ---------------------------------------------------------*/
CONTROL_DEF trajectory_control;
arm_pid_instance_f32 gyro_pid_instance;

GPIO_InitTypeDef GPIO_InitStruct;

//extern ADC_HandleTypeDef hadc1;
float i = 0;

//extern volatile float gyro_Current_Angle;

int consigne = 0;
unsigned int Pulses[2] = {0,0};

void straightControlInit(TypeOfSensors Sensor_x)
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
	if(Sensor_x == TELEMETERS)
	{
		gyro_pid_instance.Kp = 0.02;
		gyro_pid_instance.Ki = 0.000001;//0.1;
		gyro_pid_instance.Kd = 0.4;
		trajectory_control.pid_instance = &gyro_pid_instance;
		pidControllerInit(trajectory_control.pid_instance);
	}
}

void straightControl_IT(void)
{
	static int consigne = 10;
	static int front_telemeters = 0;
	static int i = 0;

	front_telemeters = (telemeters.left_front.telemeter_value + telemeters.right_front.telemeter_value) / 2;

	if (front_telemeters > 150)
	{
		motorsBrake();
		if (i < 5000)
		{
			i++;
			return;
		}

		motorSet(&left_motor, DIRECTION_FORWARD, 30, DECAY_FAST);
		motorSet(&right_motor, DIRECTION_BACKWARD, 30, DECAY_FAST);
	}
	else
	{
		i = 0;
		trajectory_control.error_val = telemeters.left_diag.telemeter_value - telemeters.right_diag.telemeter_value;
		trajectory_control.get_correction = pidController(trajectory_control.pid_instance, trajectory_control.error_val);

		Pulses[1] = consigne - trajectory_control.get_correction;
		Pulses[0] = consigne + trajectory_control.get_correction;

		//	Pulses[0] = consigne;
		//	Pulses[1] = consigne;

		motorSet(&left_motor, DIRECTION_FORWARD, Pulses[0], DECAY_FAST);
		motorSet(&right_motor, DIRECTION_FORWARD, Pulses[1], DECAY_FAST);
	}
}

void straightControlTest(void)
{
	motorsInit();
	encodersInit();
	speedControl_Init();
//	telemetersInit();
//	straightControlInit(TELEMETERS);
//	control.start_state = TRUE;
	motorsSleepDriver(OFF);
//
	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  25, "Pulses[0] =  ",(uint16_t) Pulses[0], &Font_5x8);
		ssd1306PrintInt(10,  35, "Pulses[1] =  ",(uint16_t) Pulses[1], &Font_5x8);
		ssd1306Refresh();
	}
	antiBounceJoystick();
	control.start_state = FALSE;
	motorsSleepDriver(ON);
}
