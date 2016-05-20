/**************************************************************************/
/*!
 @file    spyWall.c
 @author  PLF (PACABOT)
 @date    18 April 2016
 @version  1.1
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
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/display/pictures.h"
#include "middleware/moves/mazeMoves/mazeMoves.h"
#include "middleware/moves/basicMoves/basicMoves.h"
#include "middleware/settings/settings.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/tone/tone.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/motors/motors.h"

/* Declarations for this module */
#include "middleware/moves/mazeMoves/spyWall.h"

/* This function returns the maintain loop count according to front wall detection to avoid early turns leading to wall collision.
 *  void
 */
int spyWallGetFrontDist(spyWallGetOffsetsStruct *offset)
{
    while (hasMoveEnded() != TRUE);
    if (mainControlGetFollowType() != WALL_FOLLOW)
    {
        offset->front_dist = 0;
        return SPYWALL_E_SUCCESS;
    }

    if (getWallPresence(FRONT_WALL) == TRUE)
    {
        //        bluetoothPrintf(", FRONT DETECTED");
        if (getWallPresence(LEFT_WALL) == TRUE && getWallPresence(RIGHT_WALL) == TRUE)
        {
            offset->front_dist = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) - (zhonxCalib_data->spyWall.calib_value);
        }
        else if (getWallPresence(LEFT_WALL) == TRUE)
        {
            offset->front_dist = getTelemeterDist(TELEMETER_FL) - zhonxCalib_data->spyWall.calib_value;
        }
        else if (getWallPresence(RIGHT_WALL) == TRUE)
        {
            offset->front_dist = getTelemeterDist(TELEMETER_FR) - zhonxCalib_data->spyWall.calib_value;
        }
        else
        {
            offset->front_dist = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) - ((zhonxCalib_data->spyWall.calib_value) + 20.00);
            if (fabs(offset->front_dist) > (2.00 * OFFSET_DIST))
            {
                if (offset->front_dist > 0.00)
                    offset->front_dist = OFFSET_DIST;
                else
                    offset->front_dist = -OFFSET_DIST;
            }
        }
    }
    else
        offset->front_dist = 0;
    return SPYWALL_E_SUCCESS;
}

void spyWallFrontDistCal(void)
{

    int rv;
    double medium_dist = 0.0;
    double right_dist = 0;
    double left_dist = 0;
    int max_speed = 50;
    int end_speed = 50;
    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawBmp(frontCal_Img, 25, 24, 74, 31);
        ssd1306DrawStringAtLine(30, 0, "SPYWALL CALIBRATION", &Font_3x6);
        ssd1306Refresh();
        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(10);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(40, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    wallFollowSetInitialPosition(0.00); //absolute position into a cell

    HAL_Delay(2000);
    telemetersStart();
    ssd1306ClearScreen(MAIN_AREA);

    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(0); //absolute position into a cell
    basicMove(0, MAIN_DIST + OFFSET_DIST, max_speed, max_speed); //distance with last basicMove offset

    while (hasMoveEnded() != TRUE);
    left_dist = getTelemeterDist(TELEMETER_FL);
    right_dist = getTelemeterDist(TELEMETER_FR);

    telemetersStop();
    basicMoveStop();
    motorsDriverSleep(ON);

    medium_dist = (right_dist + left_dist) / 2.00;

    ssd1306PrintfAtLine(0, 1, &Font_5x8, "left dist  = %d", (uint32_t)(left_dist * 10.00));
    ssd1306PrintfAtLine(0, 2, &Font_5x8, "right dist = %d", (uint32_t)(right_dist * 10.00));
    ssd1306PrintfAtLine(0, 3, &Font_5x8, "moy dist   = %d", (uint32_t)(medium_dist * 10.00));



    // Save the calibration value to flash memory
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&(zhonxCalib_data->spyWall.calib_value),
                     (unsigned char *)&medium_dist, sizeof(double));
    if (rv != FLASH_DRIVER_E_SUCCESS)
    {
        ssd1306PrintfAtLine(0, 1, &Font_5x8, "FAILED To write calibration value");
        ssd1306Refresh();
        HAL_Delay(2000);
    }

    ssd1306Refresh();

    while (expanderJoyFiltered() != JOY_LEFT);
    return;
}

void spyWallFrontTest(void)
{
    uint32_t Vmin, Vmax, Vrotate;

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    wallFollowSetInitialPosition(0.00); //absolute position into a cell

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawBmp(frontTest_Img, 25, 24, 74, 31);
        ssd1306DrawStringAtLine(5, 0, "SPYWALL TEST", &Font_3x6);
        ssd1306Refresh();

        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(10);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    HAL_Delay(2000);
    ssd1306ClearScreen(MAIN_AREA);
    telemetersStart();

    Vmin = 100;
    Vmax = 100;

    basicMove(0, OFFSET_DIST, Vmax, Vmax);
    while (hasMoveEnded() != TRUE);
    mazeMoveCell(1, Vmax, Vmin);
    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
}
