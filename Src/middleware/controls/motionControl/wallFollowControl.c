/**************************************************************************/
/*!
    @file    followControl.c
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
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/gyroscope/adxrs620.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/positionControl.h"

/* Application declarations */
#include "application/lineFollower/lineFollower.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/wallFollowControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
wall_follow_control_struct wall_follow_control;
wall_follow_params_struct wall_follow_params;
arm_pid_instance_f32 telemeters_pid_instance;

int wallFollowControlInit(void)
{
	telemeters_pid_instance.Kp = 20;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 300;

	wall_follow_control.follow_pid.instance = &telemeters_pid_instance;

	wall_follow_control.succes = FALSE;

	pidControllerInit(wall_follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

char isDeadZone(void)
{
	double distance = (((encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder)) / 2.00) + OFFSET_DIST);

	if (distance > (DEADZONE_DIST - (DEADZONE / 2.00)) &&
			distance < (DEADZONE_DIST + (DEADZONE / 2.00)))
	{
		setCellState();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, SET);
		return TRUE;
	}
	//	else if (distance < (DEADZONE_DIST - (DEADZONE / 2.00)))
	//	{
	//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
	////		setCellState();
	//		return FALSE;
	//	}
	else if (distance <= 4.00) //todo add define
	{
		setCellState();
		return FALSE;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
		return FALSE;
	}
}

int wallFollowControlLoop(void)
{
	if (move_params.moveType != STRAIGHT)
		return WALL_FOLLOW_CONTROL_E_SUCCESS;

	if (isDeadZone() == TRUE) //todo redefine call architecture
	{
		position_control.position_type = POSITION_CTRL;
		wall_follow_control.follow_error = 0;
		pidControllerReset(wall_follow_control.follow_pid.instance);
	}
	else if (cell_state.left == WALL_PRESENCE)
	{
		wall_follow_control.follow_error = wallFollow(&telemeters.DL);
	}
	else if (cell_state.right == WALL_PRESENCE)
	{
		wall_follow_control.follow_error = -1.00 * wallFollow(&telemeters.DR);
	}
	else
	{
		position_control.position_type = POSITION_CTRL;
		wall_follow_control.follow_error = 0;
		pidControllerReset(wall_follow_control.follow_pid.instance);
	}

	if (fabs(wall_follow_control.follow_error) > MAX_FOLLOW_ERROR)
	{
		int sign = SIGN(wall_follow_control.follow_error);
		wall_follow_control.follow_error = MAX_FOLLOW_ERROR * sign;
	}

	wall_follow_control.follow_command = (pidController(wall_follow_control.follow_pid.instance, wall_follow_control.follow_error));

	//	volatile int telemeters_measure_error = (telemeters.it_cnt - telemeters.end_of_conversion);

	return WALL_FOLLOW_CONTROL_E_SUCCESS;
}

double wallFollow(telemeterStruct * telemeter)
{
	position_control.position_type = NO_POSITION_CTRL;
	wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double)telemeter->dist_mm;
	if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
	{
		wall_follow_control.succes = TRUE;
	}
	return DIAG_DIST_FOR_FOLLOW - (double)telemeter->dist_mm;
	return 0;
}
