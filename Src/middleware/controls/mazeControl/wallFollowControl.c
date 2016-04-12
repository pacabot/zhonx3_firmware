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
#define SUCCES_GAP_DIST 	 2.0
#define DIAG_DIST_FOR_FOLLOW 82
#define MAX_FOLLOW_ERROR	 100.00	//Millimeter

typedef struct
{
    double follow_error;
    double follow_command;
    char succes;
    pid_control_struct follow_pid;
} wall_follow_control_struct;

/* App definitions */
/* Macros */
/* Static functions */
/* extern variables */

/* global variables */
static wall_follow_control_struct wall_follow_control;
static arm_pid_instance_f32 telemeters_pid_instance;

int wallFollowControlInit(void)
{
    telemeters_pid_instance.Kp = 20;
    telemeters_pid_instance.Ki = 0;
    telemeters_pid_instance.Kd = 50;

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
    {
        expanderSetLeds(0b000);
        return WALL_FOLLOW_CONTROL_E_SUCCESS;
    }

    switch (repositionGetTelemeterUsed())
    {
        case NO_SIDE:
            positionControlEnablePositionCtrl(POSITION_CTRL);
            wall_follow_control.follow_error = 0;
            pidControllerReset(wall_follow_control.follow_pid.instance);
            expanderSetLeds(0b000);
            break;
        case ALL_SIDE:
            positionControlEnablePositionCtrl(NO_POSITION_CTRL);
            wall_follow_control.follow_error = (double) getTelemeterDist(TELEMETER_DR)
                    - (double) getTelemeterDist(TELEMETER_DL);
            expanderSetLeds(0b101);
            break;
        case LEFT_SIDE:
            positionControlEnablePositionCtrl(NO_POSITION_CTRL);
            wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double) getTelemeterDist(TELEMETER_DL);
            expanderSetLeds(0b100);
            break;
        case RIGHT_SIDE:
            positionControlEnablePositionCtrl(NO_POSITION_CTRL);
            wall_follow_control.follow_error = -1.00 * (DIAG_DIST_FOR_FOLLOW - (double) getTelemeterDist(TELEMETER_DR));
            expanderSetLeds(0b001);
            break;
    }

    if (getWallPresence(FRONT_WALL) == TRUE)
    {
        positionControlEnablePositionCtrl(POSITION_CTRL);
        wall_follow_control.follow_error = 0;
        pidControllerReset(wall_follow_control.follow_pid.instance);
    }

    if (fabs(wall_follow_control.follow_error) > MAX_FOLLOW_ERROR)
    {
        int sign = SIGN(wall_follow_control.follow_error);
        wall_follow_control.follow_error = MAX_FOLLOW_ERROR * sign;
    }

    wall_follow_control.follow_command = (pidController(wall_follow_control.follow_pid.instance,
                                                        wall_follow_control.follow_error));

    return WALL_FOLLOW_CONTROL_E_SUCCESS;
}
