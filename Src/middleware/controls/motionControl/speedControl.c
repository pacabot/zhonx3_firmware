/**************************************************************************/
/*!
    @file    speedControl.c
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
#include "peripherals/encoders/ie512.h"
#include "peripherals/motors/motors.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/speedControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */
speed_control_struct speed_control;
CONTROL_DEF speed_control_pid;
arm_pid_instance_f32 encoder_pid_instance;

static int Pulses[2] = {0,0};

int speedControl_Init(void)
{
	int rv;
	encoder_pid_instance.Kp = 2.0;
	encoder_pid_instance.Ki = 0;//0.000001;//0.1;
	encoder_pid_instance.Kd = 0;//0.4;

	speed_control.current_distance = 0;
	speed_control.gap_distance_per_loop = 0;
	speed_control.old_distance = 0;
	speed_control.current_speed = 0;

	speed_control.speed_error = 0;
	speed_control.correction = 0;
	speed_control.speed_consigne = 0;

	speed_control.speed.pid_instance = &encoder_pid_instance;

	pidControllerInit(speed_control.speed.pid_instance);


	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControl(void)
{
	int rv;
	//	int consigne = 20;

	speed_control.current_distance = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder)) / 2;
	speed_control.gap_distance_per_loop = speed_control.current_distance - speed_control.old_distance;
	speed_control.old_distance = speed_control.current_distance;
	speed_control.current_speed = (speed_control.gap_distance_per_loop * (float)HI_TIME_FREQ);

	speed_control.speed_consigne += (ACCELERATION / (float)HI_TIME_FREQ);

	speed_control.speed_error = speed_control.speed_consigne - speed_control.current_speed;
//	speed_control.speed_error = speed_control.current_cnt + speed_control.old_cnt;

	speed_control.correction = pidController(speed_control.speed.pid_instance, speed_control.speed_error);

	Pulses[1] =  (int)speed_control.correction;
	Pulses[0] =  (int)speed_control.correction;

	if (Pulses[1] > 999)
		Pulses[1] = 999;

	if (Pulses[0] > 999)
		Pulses[0] = 999;

	if (Pulses[1] < -999)
		Pulses[1] = -999;

	if (Pulses[0] < -999)
		Pulses[0] = -999;

	motorSet(&left_motor, Pulses[0], DECAY_FAST);
	motorSet(&right_motor, Pulses[1], DECAY_FAST);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedAcc(uint32_t initial_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedDcc(uint32_t final_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedMaintain(float speed)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

void speedControlTest(void)
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
		ssd1306PrintInt(10,  5,  "gap =  ",(int) (speed_control.gap_distance_per_loop * 10), &Font_5x8);
		ssd1306PrintInt(10,  15, "speed =  ",(int) speed_control.current_speed, &Font_5x8);
		ssd1306PrintInt(10,  25, "dist =  ",(int16_t) speed_control.current_distance, &Font_5x8);
		ssd1306PrintInt(10,  35, "error =  ",(int16_t) speed_control.speed_error, &Font_5x8);
		ssd1306PrintInt(10,  45, "Pulses[0] =  ",(int16_t) Pulses[0], &Font_5x8);
		ssd1306PrintInt(10,  55, "Pulses[1] =  ",(int16_t) Pulses[1], &Font_5x8);

		ssd1306Refresh();
	}
	antiBounceJoystick();
//	control.start_state = FALSE;
	motorsSleepDriver(ON);
}
