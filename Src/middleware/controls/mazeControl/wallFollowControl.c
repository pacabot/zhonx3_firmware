/**************************************************************************/
/*!
 @file    wallFollowControl.c
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
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"

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
#define MAX_FOLLOW_ERROR	 50.00	//Millimeter

#define DEADZONE_DIST        80.00  //Distance between the start of the cell and doubt area
#define DEADZONE             100.00  //doubt area
#define GETWALLPRESENCEZONE  5.00

enum telemeters_used wallFollowGetTelemeterUsed(void);

static enum telemeters_used telemeter_used = NO_SIDE;
static double current_position = 0;

typedef struct
{
    double  follow_error;
    double  follow_command;
    char    follow_type;
    char    succes;
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
    telemeters_pid_instance.Kp = 5;
    telemeters_pid_instance.Ki = 0;
    telemeters_pid_instance.Kd = 200;

    wall_follow_control.follow_pid.instance = &telemeters_pid_instance;

    wall_follow_control.succes = FALSE;

    pidControllerInit(wall_follow_control.follow_pid.instance);
    wall_follow_control.follow_error = 0;

    return POSITION_CONTROL_E_SUCCESS;
}

double wallFollowGetCommand(void)
{
    return wall_follow_control.follow_command;
}

int wallFollowSetFollowType(enum wallFollowType wallFollowType)
{
    wall_follow_control.follow_type = wallFollowType;
    return MAIN_CONTROL_E_ERROR;
}

int wallFollowControlLoop(void)
{
    if (mainControlGetMoveType() != STRAIGHT)
    {
        expanderSetLeds(0b000);
        wall_follow_control.follow_command = 0;
        wall_follow_control.follow_error = 0;
        pidControllerReset(wall_follow_control.follow_pid.instance);
        return WALL_FOLLOW_CONTROL_E_SUCCESS;
    }

    if (wall_follow_control.follow_type == PARALLEL)
    {
        switch (wallFollowGetTelemeterUsed())
        {
            case NO_SIDE:
                wall_follow_control.follow_error = 0;
                pidControllerReset(wall_follow_control.follow_pid.instance);
                expanderSetLeds(0b000);
                break;
            case ALL_SIDE:
                wall_follow_control.follow_error = (double) getTelemeterDist(TELEMETER_DR)
                - (double) getTelemeterDist(TELEMETER_DL);
                expanderSetLeds(0b101);
                break;
            case LEFT_SIDE:
                wall_follow_control.follow_error = DIAG_DIST_FOR_FOLLOW - (double) getTelemeterDist(TELEMETER_DL);
                expanderSetLeds(0b100);
                break;
            case RIGHT_SIDE:
                wall_follow_control.follow_error = -1.00 * (DIAG_DIST_FOR_FOLLOW - (double) getTelemeterDist(TELEMETER_DR));
                expanderSetLeds(0b001);
                break;
        }
    }
    else if (wall_follow_control.follow_type == DIAGONAL)
    {
        if (getTelemeterDist(TELEMETER_FL) < 200.00)
            wall_follow_control.follow_error = -1.00 * (200 - getTelemeterDist(TELEMETER_FL));
        if (getTelemeterDist(TELEMETER_FR) < 200.00)
            wall_follow_control.follow_error = (200 - getTelemeterDist(TELEMETER_FR));
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

int wallFollowSetInitialPosition(double initial_position)
{
    current_position = initial_position;
    telemeter_used = NO_SIDE;
#ifdef DEBUG_DISPLACEMENT
    bluetoothPrintf("initial dist = %d\n", (int)initial_position);
#endif
    return WALL_FOLLOW_CONTROL_E_SUCCESS;
}

enum telemeters_used wallFollowGetTelemeterUsed(void)
{
    double distance = ((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00) + current_position;

#ifdef DEBUG_WALLFOLLOW
    static char old_telemeter_used = 0xFF;
    if (telemeter_used != old_telemeter_used)
    {
        switch (telemeter_used)
        {
            case ALL_SIDE:
                old_telemeter_used = telemeter_used;
                bluetoothPrintf("ALL_SIDE \n");
                break;
            case LEFT_SIDE:
                old_telemeter_used = telemeter_used;
                bluetoothPrintf("LEFT_SIDE \n");
                break;
            case RIGHT_SIDE:
                old_telemeter_used = telemeter_used;
                bluetoothPrintf("RIGHT_SIDE \n");
                break;
            case NO_SIDE:
                old_telemeter_used = telemeter_used;
                bluetoothPrintf("NO_SIDE \n");
                break;
        }
    }
#endif

    if ((distance > DEADZONE_DIST - (DEADZONE / 2.00)) && (distance < DEADZONE_DIST + (DEADZONE / 2.00))) //check if the robot is into the DEADZONE
        telemeter_used = NO_SIDE;
    else if (((distance > OFFSET_DIST) && (distance < OFFSET_DIST + GETWALLPRESENCEZONE))   //check if the robot is into the first wallControl zone
            || (distance > (DEADZONE_DIST + (DEADZONE / 2.00)) ))                           //check if the robot is into the second wallControl zone
    {
        if ((getWallPresence(LEFT_WALL) == TRUE) && (getWallPresence(RIGHT_WALL) == TRUE))
        {
            telemeter_used = ALL_SIDE;
        }
        else if (getWallPresence(LEFT_WALL) == TRUE)
            telemeter_used = LEFT_SIDE;
        else if (getWallPresence(RIGHT_WALL) == TRUE)
            telemeter_used = RIGHT_SIDE;
        else
            telemeter_used = NO_SIDE;
    }

    return telemeter_used;
}
