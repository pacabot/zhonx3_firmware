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

/* Middleware declarations */
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/controls/mazeControl/reposition.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/tone/tone.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "application/lineFollower/lineFollower.h"

/* Declarations for this module */
#include "middleware/controls/mazeControl/wallFollowControl.h"

/* Types definitions */
//#define CENTER_DISTANCE 	 15.0
#define SUCCES_GAP_DIST 	 2.0

//#define MAX_DIST_FOR_ALIGN 	 160.00
//#define MIN_DIST_FOR_ALIGN 	 3.00

//#define MAX_DIST_FOR_FOLLOW  120.00
//#define MIN_DIST_FOR_FOLLOW  30.00

//#define BOTH_WALL_DIST 		 180.00

#define DIAG_DIST_FOR_FOLLOW 80.00

//#define SLIP_TRANSLATION	 100

//#define MAX_ANGLE_ERROR		 30.00	//Degres
#define MAX_FOLLOW_ERROR	 50.00	//Millimeter

typedef struct
{
	double follow_error;
	double follow_command;
	char succes;
	pid_control_struct follow_pid;
}wall_follow_control_struct;

/* App definitions */

/* Macros */

/* Static functions */
static double wallFollow(enum telemeterName telemeter_name);

/* extern variables */

/* global variables */
static wall_follow_control_struct wall_follow_control;
static arm_pid_instance_f32 telemeters_pid_instance;

int wallFollowControlInit(void)
{
	telemeters_pid_instance.Kp = 5;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 200;

	wall_follow_control.follow_pid.instance = &telemeters_pid_instance;

	wall_follow_control.succes = FALSE;

	pidControllerInit(wall_follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

double wallFollowGetCommand(void)
{
	return wall_follow_control.follow_command;
}

int wallFollowControlLoop(void)
{
	if (mainControlGetWallFollowType() != STRAIGHT)
		return WALL_FOLLOW_CONTROL_E_SUCCESS;

	switch (getSensorsUsedToTrackwalls())
	{
	case NO_SIDE:
		positionControlSetPositionType(POSITION_CTRL);
		wall_follow_control.follow_error = 0;
		pidControllerReset(wall_follow_control.follow_pid.instance);
//		bluetoothPrintf("NO_SIDE \n");
		break;
	case ALL_SIDE:
		positionControlSetPositionType(NO_POSITION_CTRL);
		wall_follow_control.follow_error = (double)getTelemeterDist(TELEMETER_DR) - (double)getTelemeterDist(TELEMETER_DL);
//		bluetoothPrintf("ALL_SIDE \n");
		break;
	case LEFT_SIDE:
		positionControlSetPositionType(NO_POSITION_CTRL);
		wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double)getTelemeterDist(TELEMETER_DL);
//		bluetoothPrintf("LEFT_SIDE \n");
		break;
	case RIGHT_SIDE:
		positionControlSetPositionType(NO_POSITION_CTRL);
		wall_follow_control.follow_error = -1.00 * (DIAG_DIST_FOR_FOLLOW - (double)getTelemeterDist(TELEMETER_DR));
//		bluetoothPrintf("RIGHT_SIDE \n");
		break;
	}

	if (getWallPresence(FRONT_WALL) == TRUE)
	{
		positionControlSetPositionType(POSITION_CTRL);
		wall_follow_control.follow_error = 0;
		pidControllerReset(wall_follow_control.follow_pid.instance);
	}

	if (fabs(wall_follow_control.follow_error) > MAX_FOLLOW_ERROR)
	{
		int sign = SIGN(wall_follow_control.follow_error);
		wall_follow_control.follow_error = MAX_FOLLOW_ERROR * sign;
	}

	wall_follow_control.follow_command = (pidController(wall_follow_control.follow_pid.instance, wall_follow_control.follow_error));

	return WALL_FOLLOW_CONTROL_E_SUCCESS;
}

double wallFollow(enum telemeterName telemeter_name)
{
//	positionControlSetPositionType(NO_POSITION_CTRL);
//	if (fabs(wall_follow_control.follow_error) < SUCCES_GAP_DIST)
//	{
//		wall_follow_control.succes = TRUE;
//	}
//	return DIAG_DIST_FOR_FOLLOW - (double)getTelemeterDist(telemeter_name);
}
