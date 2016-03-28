/**************************************************************************/
/*!
 @file    repositon.c
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
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/mazeControl/basicMoves.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/tone/tone.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Declarations for this module */
#include "middleware/controls/mazeControl/reposition.h"

#define DEADZONE_DIST		 90.00	//Distance between the start of the cell and doubt area
#define DEADZONE			 50.00	//doubt area

enum telemeters_used getSensorsUsedToTrackWalls(void)
{
    static enum telemeters_used telemeter_used = NO_SIDE;

    double distance = ((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00) + mouveGetInitialPosition();

    if (distance < OFFSET_DIST + 10 || distance > (DEADZONE_DIST + (DEADZONE / 2.00)))
    {
//		if (distance < (2.00 * OFFSET_DIST))
//		{
        if ((getWallPresence(LEFT_WALL) == TRUE) && (getWallPresence(RIGHT_WALL) == TRUE))
        {
            telemeter_used = ALL_SIDE;
        }
        else if ((getWallPresence(LEFT_WALL) == TRUE) || (getWallPresence(RIGHT_WALL) == TRUE))
        {
            if (getWallPresence(LEFT_WALL) == TRUE)
                telemeter_used = LEFT_SIDE;
            else
                telemeter_used = RIGHT_SIDE;
        }
        else
            telemeter_used = NO_SIDE;
//		}
    }

    if (distance > (DEADZONE_DIST - (DEADZONE / 2.00)) && distance < (DEADZONE_DIST + (DEADZONE / 2.00)))
    {
        toneStart(F3H);
        return telemeter_used = NO_SIDE;
    }
    else
    {
        toneStop();
        return telemeter_used;
    }
}

/* This function returns the maintain loop count according to front wall detection to avoid early turns leading to wall collision.
 * 	void
 */
double repositionGetPostDist(double offset)
{
    double distance;
    if (getWallPresence(FRONT_WALL) == WALL_PRESENCE)
    {
        distance = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) + 65.00
                - (CELL_LENGTH - offset);
        // Calculating average distance detected by FR and FL Telemeters
        //bluetoothPrintf("distance = %d \n", (int)distance);
        return distance;
    }
    else
        return 0.00;
}

int frontCal(float max_speed)
{
    double relative_dist = 0.00;

    while (hasMoveEnded() != TRUE)
        ;

    if (getWallPresence(FRONT_WALL) == WALL_PRESENCE)
    {
        if (getTelemeterDist(TELEMETER_FR) > getTelemeterDist(TELEMETER_FL))
        {
            move(-30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FR) - getTelemeterDist(TELEMETER_FL))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    move(30, 0, max_speed, max_speed);
                    return 0xFF;
                }
            }
        }
        else
        {
            move(30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FL) - getTelemeterDist(TELEMETER_FR))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    move(-30, 0, max_speed, max_speed);
                    return 0xFF;
                }
            }
        }
        relative_dist = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) - 21.00;
        move(0, relative_dist, 100, 100);
    }

    return POSITION_CONTROL_E_SUCCESS;
}
