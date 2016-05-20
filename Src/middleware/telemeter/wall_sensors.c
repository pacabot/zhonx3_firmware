/*
 * wall_sensors.c
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */
/* STM32 hal library declarations */
/* General declarations */
#include <math.h>
#include <stdint.h>
#include <config/basetypes.h>
#include <config/config.h>

/* Application declarations */

/* Middleware declarations */

/* Peripheral declarations */
#include <peripherals/display/smallfonts.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/telemeters/telemeters.h>

//Declarations for this module */
#include <middleware/wall_sensors/wall_sensors.h>

int getWallPresence(wallSelectorEnum wallSelector)
{
    switch (wallSelector)
    {
        case LEFT_WALL:
            if (getTelemeterDist(TELEMETER_DL) <= DISTANCE_WALL_DIAG)
                return TRUE;
            else
                return FALSE;
        case RIGHT_WALL:
            if (getTelemeterDist(TELEMETER_DR) <= DISTANCE_WALL_DIAG)
                return TRUE;
            else
                return FALSE;
        case FRONT_WALL:
            if ((getTelemeterDist(TELEMETER_FL) <= DISTANCE_WALL_FRONT) || (getTelemeterDist(TELEMETER_FR) <= DISTANCE_WALL_FRONT))
                return TRUE;
            else
                return FALSE;
    }
    return WALL_SENSORS_E_ERROR;
}

void testWallsSensors()
{
    telemetersInit();
    telemetersStart();
    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        if (getWallPresence(FRONT_WALL) == TRUE)
        {
            ssd1306FillRect(0, 59, 54, 5);
        }
        else
        {
            ssd1306DrawRect(0, 59, 54, 5);
        }
        if (getWallPresence(LEFT_WALL) == TRUE)
        {
            ssd1306FillRect(0, 10, 5, 54);
        }
        else
        {
            ssd1306DrawRect(0, 10, 5, 54);
        }
        if (getWallPresence(RIGHT_WALL) == TRUE)
        {
            ssd1306FillRect(49, 10, 5, 54);
        }
        else
        {
            ssd1306DrawRect(49, 10, 5, 54);
        }
        ssd1306PrintIntAtLine(60, 1, "FL ", (uint32_t) (getTelemeterDist(TELEMETER_FL) * 10.00), &Font_5x8);
        ssd1306PrintIntAtLine(60, 2, "DL ", (uint32_t) (getTelemeterDist(TELEMETER_DL) * 10.00), &Font_5x8);
        ssd1306PrintIntAtLine(60, 3, "DR ", (uint32_t) (getTelemeterDist(TELEMETER_DR) * 10.00), &Font_5x8);
        ssd1306PrintIntAtLine(60, 4, "FR ", (uint32_t) (getTelemeterDist(TELEMETER_FR) * 10.00), &Font_5x8);

        ssd1306Refresh();
    }
    telemetersStop();
}

void testPostSensors()
{
    telemetersInit();
    telemetersStart();

    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);

        if (fabs(getTelemeterSpeed(TELEMETER_DL)) > 500)
        {
            ssd1306FillRect(0, 58, 5, 5);
        }
        //      else
        //      {
        //      ssd1306DrawRect(0,49,5,5);
        //      }

        //      ssd1306FillRect(0,0,5,5);
        //      ssd1306DrawRect(0,0,5,5);

        if (fabs(getTelemeterSpeed(TELEMETER_DR)) > 500)
        {
            ssd1306FillRect(49, 58, 5, 5);
        }
        //      else
        //      {
        //      ssd1306DrawRect(49,49,5,5);
        //      }

        //      ssd1306FillRect(49,49,5,5);
        //
        //      ssd1306DrawRect(49,49,5,5);
        //
        //      ssd1306FillRect(49,0,5,5);
        //
        //      ssd1306DrawRect(49,0,5,5);

        ssd1306PrintIntAtLine(55, 1, "FL", (int32_t) getTelemeterSpeed(TELEMETER_FL), &Font_5x8);
        ssd1306PrintIntAtLine(55, 2, "FR", (int32_t) getTelemeterSpeed(TELEMETER_DL), &Font_5x8);
        ssd1306PrintIntAtLine(55, 3, "DL", (int32_t) getTelemeterSpeed(TELEMETER_DR), &Font_5x8);
        ssd1306PrintIntAtLine(55, 4, "DR", (int32_t) getTelemeterSpeed(TELEMETER_FR), &Font_5x8);
        ssd1306Refresh();
    }
    telemetersStop();
}
