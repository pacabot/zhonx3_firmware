/**************************************************************************/
/*!
 @file    mainControl.c
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
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"

/* Peripheral declarations */
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/times_base/times_base.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"

typedef struct
{
    char wall_follow_state;
    char line_follow_state;
    char position_state;
    char speed_state;
} control_params_struct;

static control_params_struct control_params;
move_params_struct move_params;

int debug_1 = 0;

double ROTATION_DIAMETER;

int mainControlInit(void)
{
    pid_loop.start_state = FALSE;

    ROTATION_DIAMETER = sqrt(pow(WHEELS_DISTANCE, 2) + pow(WHEELS_SPACING, 2));

    motorsInit();
    encodersInit();
    telemetersInit();
    adxrs620Init();

    speedControlInit();
    positionControlInit();
    wallFollowControlInit();
    lineFollowControlInit();
    transfertFunctionInit();

    positionControlSetPositionType(ENCODERS);
    mainControlSetFollowType(NO_FOLLOW);

    move_params.moveType = STRAIGHT;

    control_params.wall_follow_state = 0;
    control_params.line_follow_state = 0;

    return MAIN_CONTROL_E_SUCCESS;
}

int mainControlStopPidLoop(void)
{
    pid_loop.start_state = FALSE;
    expanderSetLeds(0b000);
    return MAIN_CONTROL_E_SUCCESS;
}

int mainControl_IT(void)
{
    int rv;
    if (pid_loop.start_state == FALSE)
    {
        return MAIN_CONTROL_E_SUCCESS;
    }

    if (control_params.line_follow_state == TRUE)
    {
        rv = lineFollowControlLoop();
        if (rv != LINE_FOLLOW_CONTROL_E_SUCCESS)
            return rv;
        rv = speedControlLoop();
        if (rv != SPEED_CONTROL_E_SUCCESS)
            return rv;
        rv = transfertFunctionLoop();
        if (rv != TRANSFERT_FUNCTION_E_SUCCESS)
            return rv;
        return MAIN_CONTROL_E_SUCCESS;
    }
    else if (control_params.wall_follow_state == TRUE)
    {
        rv = wallFollowControlLoop();
        if (rv != WALL_FOLLOW_CONTROL_E_SUCCESS)
            return rv;
    }
    rv = positionControlLoop();
    if (rv != POSITION_CONTROL_E_SUCCESS)
        return rv;
    rv = speedControlLoop();
    if (rv != SPEED_CONTROL_E_SUCCESS)
        return rv;
    rv = transfertFunctionLoop();
    if (rv != TRANSFERT_FUNCTION_E_SUCCESS)
        return rv;

    return MAIN_CONTROL_E_SUCCESS;
}

int mainControlSetFollowType(enum mainControlFollowType follow_type)
{
    switch (follow_type)
    {
        case LINE_FOLLOW:
            control_params.wall_follow_state = FALSE;
            control_params.line_follow_state = TRUE;
            return MAIN_CONTROL_E_SUCCESS;
        case WALL_FOLLOW:
            control_params.line_follow_state = FALSE;
            control_params.wall_follow_state = TRUE;
            return MAIN_CONTROL_E_SUCCESS;
        case NO_FOLLOW:
            control_params.line_follow_state = FALSE;
            control_params.wall_follow_state = FALSE;
            return MAIN_CONTROL_E_SUCCESS;
    }

    return MAIN_CONTROL_E_ERROR;
}

enum mainControlFollowType mainControlGetFollowType()
{
    if (control_params.wall_follow_state == TRUE)
        return WALL_FOLLOW;
    else if (control_params.line_follow_state == TRUE)
        return LINE_FOLLOW;
    else
        return NO_FOLLOW;
}

enum mainControlWallFollowType mainControlGetWallFollowType()
{
    return move_params.moveType;
}

int moveStop(void)
{
    while (hasMoveEnded() != TRUE);
    moveStraight(0, 0, 0, 500);
    HAL_Delay(500);
    motorsBrake();
    return POSITION_CONTROL_E_SUCCESS;
}

int moveEmergencyStop(void)
{
    moveStraight(0, 0, 0, 1000);
    HAL_Delay(500);
    motorsBrake();
    return POSITION_CONTROL_E_SUCCESS;
}

int move(double angle, double radius_or_distance, double max_speed, double end_speed)
{
    double distance;
    pid_loop.start_state = FALSE; //stop contol loop

    encodersReset();
    gyroResetAngle();

    speedControlSetSign((double) SIGN(radius_or_distance));
    radius_or_distance = fabsf(radius_or_distance);

    positionControlSetSign((double) SIGN(angle));
    angle = fabsl(angle);

    if (lround(angle) == 0)
    {
        move_params.moveType = STRAIGHT;

        speedProfileCompute(radius_or_distance, max_speed, end_speed, MAX_ACCEL);
        positionProfileCompute(0, 0, max_speed);
#ifdef DEBUG_MAIN_CONTROL
        bluetoothPrintf("STRAIGHT = %d\n", (int32_t)radius_or_distance);
#endif
    }
    else
    {
        distance = fabsf((PI * (2.00 * radius_or_distance) * (angle / 360.00)));

        if (lround(radius_or_distance) == 0)
        {
            move_params.moveType = ROTATE_IN_PLACE;
            speedProfileCompute(0, max_speed, end_speed, MAX_ACCEL);
            positionProfileCompute(angle, 0, max_speed);
        }
        else
        {
            move_params.moveType = CURVE;
            positionProfileCompute(angle, speedProfileCompute(distance, max_speed, end_speed, MAX_ACCEL), max_speed);
        }
#ifdef DEBUG_MAIN_CONTROL
        bluetoothPrintf("CURVE = %d\n", (int32_t)angle);
#endif
    }

    pid_loop.start_state = TRUE;
    motorsDriverSleep(OFF);
    return POSITION_CONTROL_E_SUCCESS;
}

int moveStraight(double distance, double max_speed, double end_speed, double accel)
{
    pid_loop.start_state = FALSE; //stop contol loop

    encodersReset();
    gyroResetAngle();

    speedControlSetSign((double) SIGN(distance));
    distance = fabsf(distance);

    move_params.moveType = STRAIGHT;

    speedProfileCompute(distance, max_speed, end_speed, accel);
    positionProfileCompute(0, 0, max_speed);

    pid_loop.start_state = TRUE;
    motorsDriverSleep(OFF);
    return POSITION_CONTROL_E_SUCCESS;
}

char hasMoveEnded(void)
{
    if ((positionControlHasMoveEnded() == TRUE && speedControlHasMoveEnded() == TRUE) ||
            pid_loop.start_state == FALSE)
    {
        pid_loop.start_state = FALSE;
        motorsDriverSleep(ON);
        return TRUE;
    }
    return FALSE;
}
