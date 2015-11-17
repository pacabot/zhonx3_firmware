/*
 * telemetersCal.c
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

#include "peripherals/eeprom/24lc64.h"

/* Middleware declarations */
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "peripherals/telemeters/telemetersCal.h"

//int telemetersWithOutNoise//TODO this function
int wallSensorsCalibrationFront (void)
{
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306DrawString(0,0,"Place the robot front",&Font_5x8);
	ssd1306DrawString(0,10,"of wall and press 'RIGHT'",&Font_5x8);
	ssd1306Refresh(MAIN_AREA);
	while(expanderJoyFiltered()!=JOY_RIGHT);

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306Printf(0,0,&Font_5x8,"Calibrating front sensors");
	ssd1306Refresh(MAIN_AREA);

	mainControlInit();
	motorsDriverSleep(OFF);

	telemetersStart();
	setWallFollowControl(FALSE);

	move(0,0,0,0);
	HAL_Delay(3000);
	for(int i = 0; i < NUMBER_OF_CELL; i++)
	{
//		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//		ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
//		ssd1306Refresh(MAIN_AERA);

		telemeter_FL_profile[i]=telemeters.FL.avrg;
		telemeter_FR_profile[i]=telemeters.FR.avrg;


		move(0,-NUMBER_OF_MILLIMETER_BY_LOOP,5,5);
		while(hasMoveEnded() != TRUE);
	}
	telemetersStop();
	motorsDriverSleep(ON);

	bluetoothPrintf("\n\n\nfilterd measures :\n");
	for (int i = 0; i < NUMBER_OF_CELL; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_FL_profile[i],telemeter_FR_profile[i]);
	}
	return TELEMETERS_DRIVER_E_SUCCESS;
}
int wallSensorsCalibrationDiag (void)
{
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306DrawString(0,0,"Place the robot front",&Font_5x8);
	ssd1306DrawString(0,10,"of wall and press 'RIGHT'",&Font_5x8);
	ssd1306Refresh(MAIN_AREA);

	while(expanderJoyFiltered()!=JOY_RIGHT);

	ssd1306ProgressBar(10,10,0);
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306Printf(0,0,&Font_5x8,"Calibrating front sensors");
	ssd1306Refresh(MAIN_AREA);

	mainControlInit();
	motorsDriverSleep(OFF);

	telemetersStart();
	setWallFollowControl(FALSE);

	move(0,0,0,0);
	HAL_Delay(3000);

	for(int i=0;i<NUMBER_OF_CELL;i++)
	{
//		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//		ssd1306Refresh(MAIN_AERA);
		move(0,-sqrtf(2*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),5,5);
		while(hasMoveEnded() != TRUE);
		telemeter_DL_profile[i]=telemeters.DL.avrg;
		telemeter_DR_profile[i]=telemeters.DR.avrg;
	}
	telemetersStop();
	motorsDriverSleep(ON);

	bluetoothPrintf("\n\n\nfilterd diag measures :\n");
	for (int i = 0; i < NUMBER_OF_CELL; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_DL_profile[i],telemeter_DR_profile[i]);
	}
	return TELEMETERS_DRIVER_E_SUCCESS;
}

void testTelemeterDistance()
{
	telemetersInit();
	telemetersStart();
	motorsDriverSleep(OFF);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AREA);

		ssd1306Printf(0, 0 ,&Font_5x8,"F L :%d",(int)(telemeters.FL.dist_mm*10));
		ssd1306Printf(0, 10,&Font_5x8,"F R :%d",(int)(telemeters.FR.dist_mm*10));
		ssd1306Printf(0, 20,&Font_5x8,"D L :%d",(int)(telemeters.DL.dist_mm*10));
		ssd1306Printf(0, 30,&Font_5x8,"D R :%d",(int)(telemeters.DR.dist_mm*10));

		ssd1306Printf(60,0,&Font_5x8,"V f l=%d",(int)(telemeters.FL.avrg));
		ssd1306Printf(60,10,&Font_5x8,"V f r=%d",(int)(telemeters.FR.avrg));
		ssd1306Printf(60,20,&Font_5x8,"V d l=%d",(int)(telemeters.DL.avrg));
		ssd1306Printf(60,30,&Font_5x8,"V d r=%d",(int)(telemeters.DR.avrg));


		ssd1306Refresh(MAIN_AREA);
	}
	telemetersStop();
}

void testWallsSensors()
{
	telemetersInit();
	telemetersStart();
	motorsDriverSleep(OFF);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AREA);
		if (getWallPresence(FRONT_WALL) == WALL_PRESENCE)
		{
			ssd1306FillRect(0,49,54,5);
		}
		else
		{
			ssd1306DrawRect(0,49,54,5);
		}
//		switch (cell_state.next_front) //todo add this functionality
//		{
//		case WALL_PRESENCE:
//			ssd1306FillRect(0,0,54,5);
//			break;
//		case NO_KNOWN :
//			ssd1306DrawRect(0,0,54,5);
//			break;
//		default:
//			break;
//		}
		switch (getWallPresence(LEFT_WALL))
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
		switch (getWallPresence(RIGHT_WALL))
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
		ssd1306Refresh(MAIN_AREA);
	}
	telemetersStop();
}

void testPostSensors()
{
	telemetersInit();
	telemetersStart();


	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AREA);

		if (fabs(telemeters.DL.speed_mms) > 500)
		{
		ssd1306FillRect(0,49,5,5);
		}
//		else
//		{
//		ssd1306DrawRect(0,49,5,5);
//		}

//		ssd1306FillRect(0,0,5,5);
//		ssd1306DrawRect(0,0,5,5);

		if (fabs(telemeters.DR.speed_mms) > 500)
		{
		ssd1306FillRect(49,49,5,5);
		}
//		else
//		{
//		ssd1306DrawRect(49,49,5,5);
//		}

//		ssd1306FillRect(49,49,5,5);
//
//		ssd1306DrawRect(49,49,5,5);
//
//		ssd1306FillRect(49,0,5,5);
//
//		ssd1306DrawRect(49,0,5,5);


		ssd1306PrintInt(55, 0 , "", (int32_t) telemeters.FL.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 10, "", (int32_t) telemeters.DL.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 20, "", (int32_t) telemeters.DR.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 30, "", (int32_t) telemeters.FR.speed_mms, &Font_5x8);
		ssd1306Refresh(MAIN_AREA);
	}
	telemetersStop();
}
