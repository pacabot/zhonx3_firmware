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

void wallSensorInit(void)
{
	memset(&telemeters, 0, sizeof(telemetersStruct));
//	static int old_value_DR	= 0;
//	static int old_value_DL	= 0;
//	static int old_value_FR	= 0;
//	static int old_value_FL	= 0;
//
//	static int cell_DR		= NUMBER_OF_CELL - 1;
//	static int cell_DL		= NUMBER_OF_CELL - 1;
//	static int cell_FR		= NUMBER_OF_CELL - 1;
//	static int cell_FL		= NUMBER_OF_CELL - 1;
//
//	char sens_DR	= 1;
//	char sens_DL	= 1;
//	char sens_FR	= 1;
//	char sens_FL	= 1;
//	resetTrendDist(&telemeters);
}


walls getCellState()
{
	walls walls_position = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};

	if (telemeters.FL.dist_mm < DISTANCE_FIRST_WALL_FRONT)
	{
		walls_position.front = WALL_PRESENCE;
	}
	if (telemeters.FL.dist_mm < DISTANCE_SEGOND_WALL_FRONT)
		walls_position.next_front = WALL_PRESENCE;
	if (telemeters.DL.dist_mm < DISTANCE_WALL_DIAG)
		walls_position.left = WALL_PRESENCE;
	if (telemeters.DR.dist_mm < DISTANCE_WALL_DIAG)
		walls_position.right = WALL_PRESENCE;
	return walls_position;
	}

void testTelemeterDistance()
{
	telemetersInit();
	telemetersStart();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();

		ssd1306Printf(0, 0 ,&Font_5x8,"F L :%d",(int)(telemeters.FL.dist_mm*10));
		ssd1306Printf(0, 10,&Font_5x8,"F R :%d",(int)(telemeters.FR.dist_mm*10));
		ssd1306Printf(0, 20,&Font_5x8,"D L :%d",(int)(telemeters.DL.dist_mm*10));
		ssd1306Printf(0, 30,&Font_5x8,"D R :%d",(int)(telemeters.DR.dist_mm*10));

		ssd1306Printf(60,0,&Font_5x8,"V f l=%d",(int)(telemeters.FL.avrg));
		ssd1306Printf(60,10,&Font_5x8,"V f r=%d",(int)(telemeters.FR.avrg));
		ssd1306Printf(60,20,&Font_5x8,"V d l=%d",(int)(telemeters.DL.avrg));
		ssd1306Printf(60,30,&Font_5x8,"V d r=%d",(int)(telemeters.DR.avrg));


		ssd1306Refresh();
	}
	telemetersStop();
}

void testWallsSensors()
{
	telemetersInit();
	telemetersStart();
	wallSensorInit();
	walls wall_saw;

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		wall_saw=getCellState();
		ssd1306ClearScreen();
		if (wall_saw.front == WALL_PRESENCE)
		{
			ssd1306FillRect(0,49,54,5);
		}
		else
		{
			ssd1306DrawRect(0,49,54,5);
		}
		switch (wall_saw.next_front)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(0,0,54,5);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(0,0,54,5);
				break;
			default:
				break;
		}
		switch (wall_saw.left)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(0,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(0,0,5,54);
				break;
			default:
				break;
		}
		switch (wall_saw.right)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(49,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(49,0,5,54);
				break;
			default:
				break;
		}
		ssd1306Printf(55, 0 ,&Font_5x8,"F L :%d",(int)(telemeters.FL.dist_mm*10));
		ssd1306Printf(55, 10,&Font_5x8,"F R :%d",(int)(telemeters.FR.dist_mm*10));
		ssd1306Printf(55, 20,&Font_5x8,"D L :%d",(int)(telemeters.DL.dist_mm*10));
		ssd1306Printf(55, 30,&Font_5x8,"D R :%d",(int)(telemeters.DR.dist_mm*10));
		ssd1306Refresh();
	}
	telemetersStop();
}

telemetersStruct * getDistance_ptr(void)
{
	return &telemeters;
}
