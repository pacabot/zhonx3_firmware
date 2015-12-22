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
#include "peripherals/flash/flash.h"

/* Middleware declarations */
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "peripherals/telemeters/telemetersCal.h"

//int telemetersWithOutNoise//TODO this function

int wallSensorsCalibrationFront(void)
{
    FRONT_TELEMETERS_PROFILE front_telemeters;
    int i;
    int rv;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawString(0, 0, "Place the robot front", &Font_5x8);
    ssd1306DrawString(0, 10, "of wall and press 'RIGHT'", &Font_5x8);
    ssd1306Refresh(MAIN_AREA);
    while(expanderJoyFiltered() != JOY_RIGHT);

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Printf(0, 0, &Font_5x8, "Calibrating front sensors");
    ssd1306Refresh(MAIN_AREA);

    mainControlInit();
    motorsDriverSleep(OFF);

    telemetersStart();
    mainControlSetFollowType(FALSE);

    move(0, 0, 0, 0);
    HAL_Delay(3000);
    for(i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; i++)
    {
//        ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//        ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
//        ssd1306Refresh(MAIN_AERA);

//        telemeter_FL_profile[i]=getTelemeterAvrg(TELEMETER_FL);
        front_telemeters.left[i]  = getTelemeterAvrg(TELEMETER_FL);
//        telemeter_FR_profile[i]=getTelemeterAvrg(TELEMETER_FR);
        front_telemeters.right[i] = getTelemeterAvrg(TELEMETER_FR);

        move(0, -NUMBER_OF_MILLIMETER_BY_LOOP, 5, 5);
        while(hasMoveEnded() != TRUE);
    }

    telemetersStop();
    motorsDriverSleep(ON);

    bluetoothPrintf("\n\n\nfilterd measures :\n");
    for (int i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; ++i)
    {
//        bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_FL_profile[i],telemeter_FR_profile[i]);
        bluetoothPrintf("%d|%d|%d\n", i,
                        front_telemeters.left[i], front_telemeters.right[i]);
    }

    bluetoothPrintf("Saving Front telemeters profile into Flash memory...\n");
    // Write telemeters profiles in Flash memory
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&telemeters_profile->front,
                     (unsigned char *)&front_telemeters,
                     sizeof(FRONT_TELEMETERS_PROFILE));
    if (rv == FLASH_E_SUCCESS)
    {
        bluetoothPrintf("Values saved into Flash Memory\n");
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306Printf(0, 0, &Font_5x8, "FLASH memory updated");
        ssd1306Refresh(MAIN_AREA);
    }
    else
    {
        bluetoothPrintf("Failed to write Flash Memory (%d)\n", rv);
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306Printf(0, 0, &Font_5x8, "FLASH write error (%d)", rv);
        ssd1306Refresh(MAIN_AREA);
    }

    return TELEMETERS_DRIVER_E_SUCCESS;
}

int wallSensorsCalibrationDiag (void)
{
    DIAG_TELEMETERS_PROFILE diag_telemeters;
    int i;
    int rv;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawString(0, 0, "Place the robot front", &Font_5x8);
    ssd1306DrawString(0, 10, "of wall and press 'RIGHT'", &Font_5x8);
    ssd1306Refresh(MAIN_AREA);

    while(expanderJoyFiltered()!=JOY_RIGHT);

    ssd1306ProgressBar(10, 10, 0);
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Printf(0, 0, &Font_5x8, "Calibrating front sensors");
    ssd1306Refresh(MAIN_AREA);

    mainControlInit();
    motorsDriverSleep(OFF);

    telemetersStart();
    mainControlSetFollowType(FALSE);

    move(0, 0, 0, 0);
    HAL_Delay(3000);

    for(i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; i++)
    {
//        ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//        ssd1306Refresh(MAIN_AERA);
        move(0, -sqrtf(2 * powf(NUMBER_OF_MILLIMETER_BY_LOOP, 2)), 5, 5);
        while(hasMoveEnded() != TRUE);

//        telemeter_DL_profile[i]=getTelemeterAvrg(TELEMETER_DL);
//        telemeter_DR_profile[i]=getTelemeterAvrg(TELEMETER_DR);
        diag_telemeters.left[i]  = getTelemeterAvrg(TELEMETER_DL);
        diag_telemeters.right[i] = getTelemeterAvrg(TELEMETER_DR);
    }
    telemetersStop();
    motorsDriverSleep(ON);

    bluetoothPrintf("\n\n\nfilterd diag measures :\n");
    for (int i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; ++i)
    {
//        bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_DL_profile[i],telemeter_DR_profile[i]);
        bluetoothPrintf("%d|%d|%d\n", i,
                        diag_telemeters.left[i], diag_telemeters.right[i]);
    }

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Printf(0, 0, &Font_5x8, "Writing FLASH...");
    ssd1306Refresh(MAIN_AREA);

    bluetoothPrintf("Saving Diagonal telemeters profile into Flash memory...\n");
    // Write telemeters profiles in Flash memory
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&telemeters_profile->diag,
                     (unsigned char *)&diag_telemeters,
                     sizeof(DIAG_TELEMETERS_PROFILE));
    if (rv == FLASH_E_SUCCESS)
    {
        bluetoothPrintf("Values saved into Flash Memory\n");
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306Printf(0, 0, &Font_5x8, "FLASH memory updated");
        ssd1306Refresh(MAIN_AREA);
    }
    else
    {
        bluetoothPrintf("Failed to write Flash Memory (%d)\n", rv);
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306Printf(0, 0, &Font_5x8, "FLASH write error (%d)", rv);
        ssd1306Refresh(MAIN_AREA);
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

		ssd1306Printf(0, 0 ,&Font_5x8,"F L :%d",(int)(getTelemeterDist(TELEMETER_FL)*10));
		ssd1306Printf(0, 10,&Font_5x8,"F R :%d",(int)(getTelemeterDist(TELEMETER_FR)*10));
		ssd1306Printf(0, 20,&Font_5x8,"D L :%d",(int)(getTelemeterDist(TELEMETER_DL)*10));
		ssd1306Printf(0, 30,&Font_5x8,"D R :%d",(int)(getTelemeterDist(TELEMETER_DR)*10));

		ssd1306Printf(60,0,&Font_5x8,"V f l=%d",(int)(getTelemeterAvrg(TELEMETER_FL)));
		ssd1306Printf(60,10,&Font_5x8,"V f r=%d",(int)(getTelemeterAvrg(TELEMETER_FR)));
		ssd1306Printf(60,20,&Font_5x8,"V d l=%d",(int)(getTelemeterAvrg(TELEMETER_DL)));
		ssd1306Printf(60,30,&Font_5x8,"V d r=%d",(int)(getTelemeterAvrg(TELEMETER_DR)));

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
		ssd1306Printf(55, 0 ,&Font_5x8,"F L :%d",(int)(getTelemeterDist(TELEMETER_FL)*10));
		ssd1306Printf(55, 10,&Font_5x8,"F R :%d",(int)(getTelemeterDist(TELEMETER_FR)*10));
		ssd1306Printf(55, 20,&Font_5x8,"D L :%d",(int)(getTelemeterDist(TELEMETER_DL)*10));
		ssd1306Printf(55, 30,&Font_5x8,"D R :%d",(int)(getTelemeterDist(TELEMETER_DR)*10));
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

		if (fabs(getTelemeterSpeed(TELEMETER_DL)) > 500)
		{
		ssd1306FillRect(0,49,5,5);
		}
//		else
//		{
//		ssd1306DrawRect(0,49,5,5);
//		}

//		ssd1306FillRect(0,0,5,5);
//		ssd1306DrawRect(0,0,5,5);

		if (fabs(getTelemeterSpeed(TELEMETER_DR)) > 500)
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


		ssd1306PrintInt(55, 0 , "", (int32_t) getTelemeterSpeed(TELEMETER_FL), &Font_5x8);
		ssd1306PrintInt(55, 10, "", (int32_t) getTelemeterSpeed(TELEMETER_DL), &Font_5x8);
		ssd1306PrintInt(55, 20, "", (int32_t) getTelemeterSpeed(TELEMETER_DR), &Font_5x8);
		ssd1306PrintInt(55, 30, "", (int32_t) getTelemeterSpeed(TELEMETER_FR), &Font_5x8);
		ssd1306Refresh(MAIN_AREA);
	}
	telemetersStop();
}
