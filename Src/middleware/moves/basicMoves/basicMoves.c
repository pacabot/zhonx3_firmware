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

/* Declarations for this module */
#include "middleware/moves/basicMoves/basicMoves.h"

int basicMoveStop(void)
{
    while (hasMoveEnded() != TRUE);
    basicMove(0, 0, 0, 0);
    HAL_Delay(300);
    motorsBrake();
    return POSITION_CONTROL_E_SUCCESS;
}

int basicMove(double angle, double radius_or_distance, double max_speed, double end_speed)
{
    double distance;
    //    mainControlStopPidLoop(); //stop contol loop

    encodersReset();
    gyroResetAngle();

    speedControlSetSign((double) SIGN(radius_or_distance));
    radius_or_distance = fabsf(radius_or_distance);

    positionControlSetSign((double) SIGN(angle));
    angle = fabsl(angle);

    if (lround(angle) == 0)
    {
        if (lround(radius_or_distance) == 0)
            mainControlSetMoveType(ROTATE_IN_PLACE);
        else
            mainControlSetMoveType(STRAIGHT);

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
            mainControlSetMoveType(ROTATE_IN_PLACE);
            speedProfileCompute(0, max_speed, end_speed, MAX_ACCEL);
            positionProfileCompute(angle, 0, max_speed);
        }
        else
        {
            mainControlSetMoveType(CURVE);
            positionProfileCompute(angle, speedProfileCompute(distance, max_speed, end_speed, MAX_ACCEL), max_speed);
        }
#ifdef DEBUG_MAIN_CONTROL
        bluetoothPrintf("CURVE = %d\n", (int32_t)angle);
#endif
    }

    mainControlSartPidLoop();
    motorsDriverSleep(OFF);
    return POSITION_CONTROL_E_SUCCESS;
}

int basicMoveStraight(double distance, double max_speed, double end_speed, double accel)
{
    mainControlStopPidLoop(); //stop contol loop

    encodersReset();
    gyroResetAngle();

    speedControlSetSign((double) SIGN(distance));
    distance = fabsf(distance);

    mainControlSetMoveType(STRAIGHT);

    speedProfileCompute(distance, max_speed, end_speed, accel);
    positionProfileCompute(0, 0, max_speed);

    mainControlSartPidLoop();
    motorsDriverSleep(OFF);
    return POSITION_CONTROL_E_SUCCESS;
}

int basicMoveClothoid(double distance, double max_speed, double end_speed, double accel)
{
    mainControlStopPidLoop(); //stop contol loop

    encodersReset();
    gyroResetAngle();

    speedControlSetSign((double) SIGN(distance));
    distance = fabsf(distance);

    mainControlSetMoveType(CLOTHOID);

    speedProfileCompute(distance, max_speed, end_speed, accel);
    positionProfileCompute(0, 0, 0);

    mainControlSartPidLoop();
    motorsDriverSleep(OFF);
    return POSITION_CONTROL_E_SUCCESS;
}
