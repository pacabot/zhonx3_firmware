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
#include "middleware/controls/motionControl/followControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
follow_control_struct follow_control;
follow_params_struct follow_params;
arm_pid_instance_f32 telemeters_pid_instance;

int followControlInit(void)
{
	telemeters_pid_instance.Kp = 15;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 50;

	follow_control.follow_pid.instance = &telemeters_pid_instance;

	follow_control.succes = 0;
	follow_control.follow_type = 0;

	pidControllerInit(follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int followControlLoop(void)
{
	telemetersDistancesTypeDef distances;

	if (follow_control.follow_type == ALIGN_FRONT)
	{
		getTelemetersDistance(&distances);
		if ((distances.distance_front_left < MAX_DIST_FOR_ALIGN && distances.distance_front_right < MAX_DIST_FOR_ALIGN) &&
				(distances.distance_front_left > MIN_DIST_FOR_ALIGN && distances.distance_front_right > MIN_DIST_FOR_ALIGN))
		{
			follow_control.follow_error = distances.distance_front_left - distances.distance_front_right;
			if (follow_control.follow_error < SUCCES_GAP_DIST)
			{
				follow_control.succes = TRUE;
			}
		}
		else
		{
			follow_control.follow_error = 0;
			follow_control.succes = FALSE;
		}
	}
	if (follow_control.follow_type == FOLLOW_WALL)
	{
		walls wall_saw;
		wall_saw = getCellState();
		getTelemetersDistance(&distances);
		if (wall_saw.left == WALL_PRESENCE && wall_saw.right == WALL_PRESENCE)
		{
			bothWallFollow(&distances);
		}
		else if (wall_saw.left == WALL_PRESENCE)
		{
			leftWallFollow(&distances);
		}
		else if (wall_saw.right == WALL_PRESENCE)
		{
			rightWallFollow(&distances);
		}
	}
	if (follow_control.follow_type == FOLLOW_LINE)
	{
		walls wall_saw;
		wall_saw = getCellState();
		getTelemetersDistance(&distances);
		if (wall_saw.left == WALL_PRESENCE && wall_saw.right == WALL_PRESENCE)
		{
			bothWallFollow(&distances);
		}
		else if (wall_saw.left == WALL_PRESENCE)
		{
			leftWallFollow(&distances);
		}
		else if (wall_saw.right == WALL_PRESENCE)
		{
			rightWallFollow(&distances);
		}
	}

	follow_control.follow_command = (pidController(follow_control.follow_pid.instance, follow_control.follow_error));

	return SPEED_CONTROL_E_SUCCESS;
}

int bothWallFollow(telemetersDistancesTypeDef *distances)
{
	if ((distances->distance_diag_left < MAX_DIST_FOR_FOLLOW && distances->distance_diag_right < MAX_DIST_FOR_FOLLOW) &&
			(distances->distance_diag_left > MIN_DIST_FOR_FOLLOW && distances->distance_diag_right > MIN_DIST_FOR_FOLLOW))
	{
		follow_control.follow_error = distances->distance_diag_right - distances->distance_diag_left;
		if (follow_control.follow_error < SUCCES_GAP_DIST)
		{
			follow_control.succes = TRUE;
		}
	}
	else
	{
		follow_control.follow_error = 0;
		follow_control.succes = FALSE;
	}
	return SPEED_CONTROL_E_SUCCESS;
}

int rightWallFollow(telemetersDistancesTypeDef *distances)
{
	if (distances->distance_diag_right < MAX_DIST_FOR_FOLLOW && distances->distance_diag_right > MIN_DIST_FOR_FOLLOW)
	{
		follow_control.follow_error = (double)distances->distance_diag_right - DIAG_DIST_FOR_FOLLOW;
		if (follow_control.follow_error < SUCCES_GAP_DIST)
		{
			follow_control.succes = TRUE;
		}
	}
	else
	{
		follow_control.follow_error = 0;
		follow_control.succes = FALSE;
	}
	return SPEED_CONTROL_E_SUCCESS;
}

int leftWallFollow(telemetersDistancesTypeDef *distances)
{
	if (distances->distance_diag_left < MAX_DIST_FOR_FOLLOW && distances->distance_diag_left > MIN_DIST_FOR_FOLLOW)
	{
		follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double)distances->distance_diag_left;
		if (follow_control.follow_error < SUCCES_GAP_DIST)
		{
			follow_control.succes = TRUE;
		}
	}
	else
	{
		follow_control.follow_error = 0;
		follow_control.succes = FALSE;
	}
	return SPEED_CONTROL_E_SUCCESS;
}

