/**************************************************************************/
/*!
 @file    repositon.c
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
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/mazeControl/basicMoves.h"
#include "middleware/display/pictures.h"

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
#include "middleware/controls/mazeControl/reposition.h"

#define DEADZONE_DIST		 80.00	//Distance between the start of the cell and doubt area
#define DEADZONE			 90.00	//doubt area
#define GETWALLPRESENCEZONE  5.00

static enum telemeters_used telemeter_used = NO_SIDE;
static double current_position = 0;

void repositionSetInitialPosition(double initial_position)
{
    current_position = initial_position;
    telemeter_used = NO_SIDE;
#ifdef DEBUG_DISPLACEMENT
    bluetoothPrintf("initial dist = %d\n", (int)initial_position);
#endif
}

enum telemeters_used repositionGetTelemeterUsed(void)
{
    double distance = ((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00) + current_position;

#ifdef DEBUG_REPOSITION
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

    if ((distance > DEADZONE_DIST - (DEADZONE / 2.00)) && (distance < DEADZONE_DIST + (DEADZONE / 2.00)))
        telemeter_used = NO_SIDE;
    else if (((distance > OFFSET_DIST) && (distance < OFFSET_DIST + GETWALLPRESENCEZONE))
            || (distance > (DEADZONE_DIST + (DEADZONE / 2.00)) ))
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

/* This function returns the maintain loop count according to front wall detection to avoid early turns leading to wall collision.
 *  void
 */
int repositionGetFrontDist(repositionGetOffsetsStruct *offset)
{
    while (hasMoveEnded() != TRUE);
    if (getWallPresence(FRONT_WALL) == WALL_PRESENCE)
    {
//        if (getWallPresence(LEFT_WALL) == WALL_PRESENCE && getWallPresence(RIGHT_WALL) == WALL_PRESENCE)
//        {
        offset->front_dist = (int)(getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) - (zhonxCalib_data->reposition.calib_value * 2);
//        }
//        if (getWallPresence(LEFT_WALL) == WALL_PRESENCE)
//        {
//            error_distance = (int)(getTelemeterDist(TELEMETER_FR)) - zhonxCalib_data->reposition.calib_value;
//        }
//        else if (getWallPresence(RIGHT_WALL) == WALL_PRESENCE)
//        {
//            error_distance = (int)(getTelemeterDist(TELEMETER_FL)) - zhonxCalib_data->reposition.calib_value;
//        }
    }
    else
        offset->front_dist = 0;
    return REPOSITION_E_SUCCESS;
}

void repositionFrontDistCal(void)
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
        ssd1306DrawStringAtLine(30, 0, "FRONT CALIBRATION", &Font_3x6);
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

    mainControlInit();
    mainControlSetFollowType(NO_FOLLOW);
    HAL_Delay(2000);
    telemetersStart();
    ssd1306ClearScreen(MAIN_AREA);

    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(0); //absolute position into a cell
    move(0, MAIN_DIST + OFFSET_DIST, max_speed, max_speed); //distance with last move offset

    while (hasMoveEnded() != TRUE);
    left_dist = getTelemeterDist(TELEMETER_FL);
    right_dist = getTelemeterDist(TELEMETER_FR);

    repositionSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
    move(0, OFFSET_DIST, max_speed, end_speed);
    while (hasMoveEnded() != TRUE);

    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);

    medium_dist = (right_dist + left_dist) / 2.00;

    ssd1306PrintfAtLine(0, 1, &Font_5x8, "left dist  = %d", (uint32_t)(left_dist * 10.00));
    ssd1306PrintfAtLine(0, 2, &Font_5x8, "right dist = %d", (uint32_t)(right_dist * 10.00));
    ssd1306PrintfAtLine(0, 3, &Font_5x8, "moy dist   = %d", (uint32_t)(medium_dist * 10.00));



    // Save the calibration value to flash memory
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&(zhonxCalib_data->reposition.calib_value),
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

void repositionFrontTest(void)
{
    uint32_t Vmin, Vmax, Vrotate;

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawBmp(frontTest_Img, 25, 24, 74, 31);
        ssd1306DrawStringAtLine(5, 0, "FRONT REPOSITION TEST", &Font_3x6);
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

    mainControlInit();
    HAL_Delay(2000);
    ssd1306ClearScreen(MAIN_AREA);
    telemetersStart();

    Vmin = 100;
    Vmax = 100;

    move(0, OFFSET_DIST, Vmax, Vmax);
    while (hasMoveEnded() != TRUE);
    moveCell(1, Vmax, Vmin);
    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
}
