/*
 * wall_sensors.c
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <middleware/controls/mainControl/mainControl.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Application declarations */
#include "application/statistiques/statistiques.h"

/* Peripheral declarations */
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/motors/motors.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"

walls cell_state = { NO_WALL, NO_WALL, NO_WALL, NO_WALL };

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
            if ((getTelemeterDist(TELEMETER_FL) <= DISTANCE_WALL_FRONT) && (getTelemeterDist(TELEMETER_FR) <= DISTANCE_WALL_FRONT))
                return TRUE;
            else
                return FALSE;
    }
    return WALL_SENSORS_E_ERROR;
}
