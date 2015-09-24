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
	telemeters_pid_instance.Kp = 10;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 300;

	wall_follow_control.follow_pid.instance = &telemeters_pid_instance;

	wall_follow_control.succes = FALSE;

	pidControllerInit(wall_follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int wallFollowControlLoop(void)
{
	if (move_params.moveType != STRAIGHT)
		return WALL_FOLLOW_CONTROL_E_SUCCESS;

	if (cell_state.left == WALL_PRESENCE)
	{
		wallFollow(&telemeters.DL);
	}
//	else if (cell_state.right == WALL_PRESENCE)
//	{
//		wallFollow(&telemeters.DR);
//	}
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


	//wall_follow_control.follow_error -= -GyroGetAngle() * (((encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder) / 2.00)/500.00)) ;


	wall_follow_control.follow_command = (pidController(wall_follow_control.follow_pid.instance, wall_follow_control.follow_error));

//	volatile int telemeters_measure_error = (telemeters.it_cnt - telemeters.end_of_conversion);

	return WALL_FOLLOW_CONTROL_E_SUCCESS;
}

int wallFollow(telemeterStruct * telemeter)
{
	if (GyroGetAngle() < 4 && fabs(telemeter->speed_mms) > 300)
	{
		wall_follow_control.follow_error = 0;
		pidControllerReset(wall_follow_control.follow_pid.instance);

		position_control.position_type = POSITION_CTRL;
//			cell_state.left = NO_WALL;
		setCellState();
	}
	else
	{
		position_control.position_type = NO_POSITION_CTRL;
		wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double)telemeter->dist_mm;
		if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
		{
			wall_follow_control.succes = TRUE;
			GyroResetAngle();
		}
	}
	return 0;
}


//int wallFollowControlLoop(void)
//{
//	telemetersStruct *ptr_distances = getDistance_ptr();
//
//
//	//	if (wall_follow_control.follow_type == ALIGN_FRONT)
//	//	{
//	//		getTelemetersDistance(ptr_distances);
//	//		if ((telemeters.FL.dist_mm < MAX_DIST_FOR_ALIGN && telemeters.FR.dist_mm < MAX_DIST_FOR_ALIGN) &&
//	//				(telemeters.FL.dist_mm > MIN_DIST_FOR_ALIGN && telemeters.FR.dist_mm > MIN_DIST_FOR_ALIGN))
//	//		{
//	//			wall_follow_control.follow_error = telemeters.FL.dist_mm - telemeters.FR.dist_mm;
//	//			if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//	//			{
//	//				wall_follow_control.succes = TRUE;
//	//			}
//	//		}
//	//		else
//	//		{
//	//			wall_follow_control.follow_error = 0;
//	//			wall_follow_control.succes = FALSE;
//	//		}
//	//	}
//	//	if (wall_follow_control.follow_type == FOLLOW_WALL)
//	//	{
//	walls wall_saw;
//	wall_saw = getCellState();
//
//	if (wall_follow_control.follow_type == FOLLOW_WALL)
//	{
//		if (wall_saw.front == WALL_PRESENCE)
//		{
//			if (wall_saw.right == WALL_PRESENCE)
//			{
//				if (telemeters.DR.dist_mm > MIN_DIST_FOR_FOLLOW && telemeters.DR.dist_mm < MAX_DIST_FOR_FOLLOW)
//				{
//					if ((telemeters.FL.dist_mm < MAX_DIST_FOR_ALIGN && telemeters.FR.dist_mm < MAX_DIST_FOR_ALIGN) &&
//							(telemeters.FL.dist_mm > MIN_DIST_FOR_ALIGN && telemeters.FR.dist_mm > MIN_DIST_FOR_ALIGN))
//					{
//						alignFront();
//					}
//				}
//			}
//			else if (wall_saw.left == WALL_PRESENCE)
//			{
//				if (telemeters.DL.dist_mm > MIN_DIST_FOR_FOLLOW && telemeters.DL.dist_mm < MAX_DIST_FOR_FOLLOW)
//				{
//					if ((telemeters.FL.dist_mm < MAX_DIST_FOR_ALIGN && telemeters.FR.dist_mm < MAX_DIST_FOR_ALIGN) &&
//							(telemeters.FL.dist_mm > MIN_DIST_FOR_ALIGN && telemeters.FR.dist_mm > MIN_DIST_FOR_ALIGN))
//					{
//						alignFront();
//					}
//				}
//			}
//		}
//		else if (wall_saw.left == WALL_PRESENCE && wall_saw.right == WALL_PRESENCE)
//		{
//			bothWallFollow();
//		}
//		else if (wall_saw.left == WALL_PRESENCE)
//		{
//			leftWallFollow();
//		}
//		else if (wall_saw.right == WALL_PRESENCE)
//		{
//			rightWallFollow();
//		}
//	}
//	else
//	{
//		wall_follow_control.follow_error = 0;
//	}
//
//	telemetersStop();
//	wall_follow_control.follow_command = (pidController(wall_follow_control.follow_pid.instance, wall_follow_control.follow_error));
//	telemetersStart();
//
//	return WALL_FOLLOW_CONTROL_E_SUCCESS;
//}
//
//int alignFront()
//{
//	if ((telemeters.FL.dist_mm < MAX_DIST_FOR_ALIGN && telemeters.FR.dist_mm < MAX_DIST_FOR_ALIGN) &&
//			(telemeters.FL.dist_mm > MIN_DIST_FOR_ALIGN && telemeters.FR.dist_mm > MIN_DIST_FOR_ALIGN))
//	{
//		wall_follow_control.follow_error = telemeters.FL.dist_mm - telemeters.FR.dist_mm;
//		if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//		{
//			wall_follow_control.succes = TRUE;
//		}
//	}
//	return WALL_FOLLOW_CONTROL_E_SUCCESS;
//}

//int bothWallFollow()
//{
//	//	if ((telemeters.DL.dist_mm < MAX_DIST_FOR_FOLLOW && telemeters.DR.dist_mm < MAX_DIST_FOR_FOLLOW) &&
//	//			(telemeters.DL.dist_mm > MIN_DIST_FOR_FOLLOW && telemeters.DR.dist_mm > MIN_DIST_FOR_FOLLOW)) // &&
//	//	if 	(((telemeters.DL.dist_mm + telemeters.DR.dist_mm) < BOTH_WALL_DIST))
//	//	{
//	wall_follow_control.follow_error = telemeters.DR.dist_mm - telemeters.DL.dist_mm;
//	if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//	{
//		wall_follow_control.succes = TRUE;
//	}
//	//	}
//	return WALL_FOLLOW_CONTROL_E_SUCCESS;
//}
//
//int rightWallFollow()
//{
//	//	if (telemeters.DR.dist_mm < MAX_DIST_FOR_FOLLOW && telemeters.DR.dist_mm > MIN_DIST_FOR_FOLLOW)
//	//	{
//	wall_follow_control.follow_error = (double)telemeters.DR.dist_mm - DIAG_DIST_FOR_FOLLOW;
//	if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//	{
//		wall_follow_control.succes = TRUE;
//	}
//	//	}
//	return WALL_FOLLOW_CONTROL_E_SUCCESS;
//}
//
//int leftWallFollow()
//{
//	//	if (telemeters.DL.dist_mm < MAX_DIST_FOR_FOLLOW && telemeters.DL.dist_mm > MIN_DIST_FOR_FOLLOW)
//	//	{
//	wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double)telemeters.DL.dist_mm;
//	if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//	{
//		wall_follow_control.succes = TRUE;
//	}
//	//	}
//	return WALL_FOLLOW_CONTROL_E_SUCCESS;
//}
