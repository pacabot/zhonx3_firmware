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
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/speedControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */
speed_control_struct speed_control;
pid_control_struct speed_control_pid;
arm_pid_instance_f32 encoder_pid_instance;

int speedControlInit(void)
{
	encoder_pid_instance.Kp = 700.0;
	encoder_pid_instance.Ki = 0;
	encoder_pid_instance.Kd = 800;

	speed_control.current_distance = 0;
	speed_control.gap_distance_per_loop = 0;
	speed_control.current_distance_consign = 0;
	speed_control.old_distance = 0;
	speed_control.current_speed = 0;

	speed_control.speed_error = 0;
	speed_control.speed_command = 0;
	speed_control.speed_consign = 0;

	speed_control.speed_pid.instance = &encoder_pid_instance;

	pidControllerInit(speed_control.speed_pid.instance);


	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControlLoop(void)
{
	speed_control.current_distance = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder)) / 2;
	speedCompute();

	speedAcc();
	speedDcc();

	speed_control.current_distance_consign += (speed_control.speed_consign / (float)HI_TIME_FREQ);

	speed_control.speed_error = speed_control.current_distance_consign - speed_control.current_distance;		//for distance control
	speed_control.speed_command = pidController(speed_control.speed_pid.instance, speed_control.speed_error);

	if (speed_control.speed_consign < 0.0)
		speed_control.speed_consign = 0;

	speed_control.old_distance = speed_control.current_distance;

	return SPEED_CONTROL_E_SUCCESS;
}

int speedAcc(void)
{
	if (speed_control.current_distance <= speed_params.accel_dist)
	{
		speed_control.speedType = ACC;
		speed_control.speed_consign += (speed_params.accel / (float)HI_TIME_FREQ);								//speed consigne (mm/s)
	}

	return SPEED_CONTROL_E_SUCCESS;
}

int speedDcc(void)
{
	if (speed_control.current_distance >= (speed_params.accel_dist + speed_params.maintain_dist))
	{
		speed_control.speedType = DCC;
		speed_control.speed_consign -= (speed_params.decel / (float)HI_TIME_FREQ);								//speed consigne (mm/s)
		if (speed_control.speed_consign <= 0.0)
			speed_control.speed_consign = 0;
	}

	return SPEED_CONTROL_E_SUCCESS;
}

int speedCompute(void)
{
	speed_control.gap_distance_per_loop = speed_control.current_distance - speed_control.old_distance;	//delta distance per loop
	speed_control.current_speed = (speed_control.gap_distance_per_loop * (float)HI_TIME_FREQ);			//actual speed (mm/s)

	return SPEED_CONTROL_E_SUCCESS;
}
