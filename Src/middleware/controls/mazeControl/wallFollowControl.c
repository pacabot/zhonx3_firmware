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

/* Declarations for this module */
#include "middleware/controls/mazeControl/wallFollowControl.h"

/* Types definitions */
#define MAX_FOLLOW_ERROR     50.00  //Millimeter

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
    telemeters_pid_instance.Kp = 10;
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

    //    if (wall_follow_control.follow_type == PARALLEL)
    //    {
    switch (wallFollowGetTelemeterUsed())
    {
        case NO_SIDE:
            wall_follow_control.follow_error = 0;
            wall_follow_control.follow_command = 0;
            pidControllerReset(wall_follow_control.follow_pid.instance);
            expanderSetLeds(0b000);
            return WALL_FOLLOW_CONTROL_E_SUCCESS;
            break;
        case ALL_SIDE:
            wall_follow_control.follow_error = (double) getTelemeterDist(TELEMETER_DR)
            - (double) getTelemeterDist(TELEMETER_DL);
            expanderSetLeds(0b101);
            break;
        case LEFT_SIDE:
            wall_follow_control.follow_error = WALL_FOLLOW_DIAG_DIST - (double) getTelemeterDist(TELEMETER_DL);
            expanderSetLeds(0b100);
            break;
        case RIGHT_SIDE:
            wall_follow_control.follow_error = -1.00 * (WALL_FOLLOW_DIAG_DIST - (double) getTelemeterDist(TELEMETER_DR));
            expanderSetLeds(0b001);
            break;
    }
    //    }
    //    else if (wall_follow_control.follow_type == DIAGONAL)
    //    {
    //        if (getTelemeterDist(TELEMETER_FL) < 200.00)
    //            wall_follow_control.follow_error = -1.00 * (200 - getTelemeterDist(TELEMETER_FL));
    //        if (getTelemeterDist(TELEMETER_FR) < 200.00)
    //            wall_follow_control.follow_error = (200 - getTelemeterDist(TELEMETER_FR));
    //    }

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
#ifdef DEBUG_DISPLACEMENT
    bluetoothPrintf("initial dist = %d\n", (int)initial_position);
#endif
    return WALL_FOLLOW_CONTROL_E_SUCCESS;
}

/* mode eclipse selection multiple => alt+shift+a
 ****************************************************************
 *  FOLLOW FIRST CONDITION :
 * _________________________
 *
 *                   DEADZONE
 *                   <------>
 *      <--------------->
 *        DEADZONE DIST
 *      +---------------+---------------+---------------+
 *              /                                       |
 *       _ _   /                                        |
 *      \   n\'                                         |
 *      /_ _u/                                          |
 *               ____                                   |
 *            CHECK & ON                                |
 *      +---------------+---------------+---------------+
 *
 *      <------->
 *   OFFSET + VIEWING
 *   DIST     OFFSET
 *
 ***************************************************************
 * NO FOLLOW CONDITION :
 * _____________________
 *
 *                   DEADZONE
 *                   <------>
 *      <--------------->
 *        DEADZONE DIST
 *      +---------------+---------------+---------------+
 *      |             /                                 |
 *      |      _ _   /                                  |
 *      |     \   n\'                                   |
 *      |     /_ _u/                                    |
 *      |            ________                           |
 *      |               OFF                             |
 *      +---------------+---------------+---------------+
 *
 *              <------->
 *              DEADZONE
 *              VIEWING
 *              OFFSET
 *
 ****************************************************************
 * FOLLOW SECOND CONDITION :
 * _________________________
 *
 *         DEADZONE DIST +
 *         (DEADZONE / 2)
 *      <------------------->
 *
 *      +---------------+---------------+---------------+
 *      |                  /                            |
 *      |           _ _   /                             |
 *      |          \   n\'                              |
 *      |          /_ _u/                               |
 *      |                    ________                   |
 *      |                       ON                      |
 *      +---------------+---------------+---------------+
 *
 *                                   DEADZONE
 *                                   <------>
 *                      <--------------->
 *                        DEADZONE DIST
 *
 */
enum telemeters_used wallFollowGetTelemeterUsed(void)
{
    double telemeters_spot_distance = 0;
    int cell_count = 0;
    double robot_distance = 0;

    robot_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;
    /* cyclic action if the robot performs many cell */
    if (robot_distance > CELL_LENGTH)
    {
        cell_count = (int)robot_distance / CELL_LENGTH;
        robot_distance = robot_distance - (((double)cell_count) * CELL_LENGTH);
    }

    //    ssd1306ClearScreen(MAIN_AREA);
    //    ssd1306PrintIntAtLine(0, 1, "dist     =  ", (int)robot_distance, &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 2, "cell cnt =  ", (int)cell_count, &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 3, "enc.     =  ", (int)((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00), &Font_5x8);
    //    ssd1306Refresh();

    telemeters_spot_distance = robot_distance + current_position + DEADZONE_VIEWING_OFFSET;

    if ((telemeters_spot_distance > DEADZONE_DIST - (DEADZONE / 2.00)) && (telemeters_spot_distance < DEADZONE_DIST + (DEADZONE / 2.00))) //check if the robot is into the DEADZONE
    {
        telemeter_used = NO_SIDE;
    }
    //    else if (((telemeters_spot_distance > (OFFSET_DIST + DEADZONE_VIEWING_OFFSET)) && (telemeters_spot_distance < (OFFSET_DIST + DEADZONE_VIEWING_OFFSET + DEADZONE_CHECKWALL_DIST)))   //check if the robot is into the first wallControl zone
    //            || (telemeters_spot_distance > (DEADZONE_DIST + (DEADZONE / 2.00)) ))                           //check if the robot is into the second wallControl zone

    else if ((telemeters_spot_distance > (DEADZONE_DIST + (DEADZONE * 0.65)) ||
            telemeters_spot_distance < (DEADZONE_DIST - (DEADZONE * 0.35)))  )
    {
        if ((getTelemeterDist(TELEMETER_DL) < WALL_FOLLOW_MAX_DIAG_DIST) && (getTelemeterDist(TELEMETER_DR) < WALL_FOLLOW_MAX_DIAG_DIST))
        {
            telemeter_used = ALL_SIDE;
        }
        else if (getTelemeterDist(TELEMETER_DL) < WALL_FOLLOW_MAX_DIAG_DIST)
            telemeter_used = LEFT_SIDE;
        else if (getTelemeterDist(TELEMETER_DR) < WALL_FOLLOW_MAX_DIAG_DIST)
            telemeter_used = RIGHT_SIDE;
        else
            telemeter_used = NO_SIDE;
    }

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

    return telemeter_used;
}

void wallFollowTest(void)
{
    double telemeters_spot_distance = 0;
    int cell_count = 0;
    double robot_distance = 0;
    int i = 0;
    wallFollowSetInitialPosition(0.00); //absolute position into a cell

    encodersReset();
    telemetersStart();

    while (expanderJoyFiltered() != JOY_LEFT)
    {
        i++;
        robot_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;
        /* cyclic action if the robot performs many cell */
        if (robot_distance > CELL_LENGTH)
        {
            cell_count = (int)robot_distance / CELL_LENGTH;
            robot_distance = robot_distance - (((double)cell_count) * CELL_LENGTH);
        }

        telemeters_spot_distance = robot_distance + current_position + DEADZONE_VIEWING_OFFSET;

        if (i % 50 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintIntAtLine(0, 0, "spot dist =  ", (int)(telemeters_spot_distance), &Font_5x8);
            ssd1306PrintIntAtLine(0, 1, "dist      =  ", (int)robot_distance, &Font_5x8);
            ssd1306PrintIntAtLine(0, 2, "cell cnt  =  ", (int)cell_count, &Font_5x8);
            ssd1306PrintIntAtLine(0, 3, "enc.      =  ", (int)((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00), &Font_5x8);
            ssd1306Refresh();
        }

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
                wall_follow_control.follow_error = WALL_FOLLOW_DIAG_DIST - (double) getTelemeterDist(TELEMETER_DL);
                expanderSetLeds(0b100);
                break;
            case RIGHT_SIDE:
                wall_follow_control.follow_error = -1.00 * (WALL_FOLLOW_DIAG_DIST - (double) getTelemeterDist(TELEMETER_DR));
                expanderSetLeds(0b001);
                break;
        }
        HAL_Delay(1);
    }
}
