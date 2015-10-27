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
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "middleware/wall_sensors/wall_sensors.h"

walls cell_state = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};

char getWallPresence(enum wallSelectorEnum wallSelector)
{
	switch (wallSelector)
	{
	case LEFT_WALL:
		if (telemeters.DL.dist_mm <= DISTANCE_WALL_DIAG)
			return TRUE;
		else
			return FALSE;
	case RIGHT_WALL:
		if (telemeters.DR.dist_mm <= DISTANCE_WALL_DIAG)
			return TRUE;
		else
			return FALSE;
	case FRONT_WALL:
		if ((telemeters.FL.dist_mm + telemeters.FR.dist_mm / 2.00) <= DISTANCE_WALL_FRONT)
			return TRUE;
		else
			return FALSE;
	}
}
