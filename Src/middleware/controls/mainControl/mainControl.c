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

static enum mainControlMoveType moveType;
static control_params_struct control_params;
static pid_loop_struct pid_loop;

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

    mainControlSetMoveType(STRAIGHT);

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

int mainControlSartPidLoop(void)
{
    pid_loop.start_state = TRUE;
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

int mainControlSetFollowType(enum mainControlFollowType followType)
{
    switch (followType)
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

enum mainControlMoveType mainControlGetMoveType()
{
    return moveType;
}

int mainControlSetMoveType(enum mainControlMoveType move_type)
{
    moveType = move_type;
    return MAIN_CONTROL_E_ERROR;
}

char hasMoveEnded(void)
{
    if ((positionControlHasMoveEnded() == TRUE && speedControlHasMoveEnded() == TRUE) ||
            pid_loop.start_state == FALSE)
    {
        pid_loop.start_state = FALSE;
//        motorsDriverSleep(ON);
        return TRUE;
    }
    return FALSE;
}
