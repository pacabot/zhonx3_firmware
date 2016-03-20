/*
 * spyPost.c
 *
 *  Created on: 19 mars 2016
 *      Author: zhonx
 */

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
#include "peripherals/encoders/ie512.h"

#include "peripherals/eeprom/24lc64.h"
#include "peripherals/flash/flash.h"

/* Middleware declarations */
#include "middleware/math/kalman_filter.h"
#include "middleware/display/pictures.h"

//Declarations for this module */
#include "middleware/controls/mazeControl/spyPost.h"

#define SPYPOST_ENCODERS_STEPS_MEASURE_MM 	1
#define SPYPOST_FIRST_CAL_DISTANCE			140//(CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS))
#define SPYPOST_ARRAY_PROFILE_LENGTH 		((SPYPOST_FIRST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	16
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	70
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	102

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM 	((SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_NBITS_SAMPLING_RESOLUTION)

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

//#define MAVRG_BUFF_SIZE 					(1 << MAVRG_NBITS)

typedef struct
{
	int right [SPYPOST_ARRAY_PROFILE_LENGTH + 1];
	int left [SPYPOST_ARRAY_PROFILE_LENGTH + 1];
} spyPostProfileStruct;

/* Static functions */
static void spyPostStartMeasure(enum telemeterName telemeter, spyPostProfileStruct *profile);
static void spyPostPrintProfile(enum telemeterName telemeter, spyPostProfileStruct *profile);

int spyPostCalibration(void)
{
	int i = 0;
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306DrawBmp(spyPost_pict1, 1, 24, 128, 40);
	while (expanderJoyFiltered()!= JOY_RIGHT)
	{
		if ( expanderJoyState() == JOY_LEFT)
			return 0;
		i++;
		if (i % 2)
			ssd1306DrawStringAtLine(20, 0, "PRESS RIGHT TO CONTINUE", &Font_3x6);
		else
			ssd1306ClearRectAtLine(0, 0, 128);
		ssd1306Refresh();
		HAL_Delay(500);
	}

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintfAtLine(0, 0, &Font_5x8, "  LEFT DIAG SENSOR");
	ssd1306PrintfAtLine(0, 1, &Font_5x8, "  1st Calibration");
	ssd1306PrintfAtLine(0, 2, &Font_5x8, " follow an end wall");
	ssd1306Refresh();

	HAL_Delay(2000);

	mainControlInit();
	mainControlSetFollowType(NO_FOLLOW);

	spyPostProfileStruct spyPostProfile_wallToNoWall;
	spyPostStartMeasure(TELEMETER_DL, &spyPostProfile_wallToNoWall);
	spyPostPrintProfile(TELEMETER_DL, &spyPostProfile_wallToNoWall);

	while(1);
	return 0;
	//	return TELEMETERS_DRIVER_E_SUCCESS;
}

void spyPostStartMeasure(enum telemeterName telemeter, spyPostProfileStruct *profile)
{
	if (telemeter != TELEMETER_DR && telemeter != TELEMETER_DL)
		return;

	int i = 0;
	int sample = 0;
	telemetersStart();

	//take the measures
	move(0, SPYPOST_FIRST_CAL_DISTANCE, 5, 5);
	for(i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
	{
		while((((int)(encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) &&
				(hasMoveEnded() != TRUE));

		ssd1306ClearScreen(MAIN_AREA);
		if (telemeter == TELEMETER_DL)
		{
			sample = (int)getTelemeterDist(TELEMETER_DL);
			if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
				profile->left[i] = 0x00;
			else
				profile->left[i] = 1 << (sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM;
			ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", profile->left[i]);
		}
		else
		{
			sample = (int)getTelemeterDist(TELEMETER_DR);
			if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
				profile->right[i] = 0x00;
			else
				profile->right[i] = 1 << (sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM;

			ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", profile->right[i]);
		}

		ssd1306PrintIntAtLine(0, 0, "wall dist :  ", sample, &Font_5x8);
		ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (int)(SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);

		ssd1306ProgressBar(10,50,(i*100)/SPYPOST_ARRAY_PROFILE_LENGTH);
		ssd1306Refresh();
	}
	while(hasMoveEnded() != TRUE);
	telemetersStop();
	motorsDriverSleep(ON);
}

void spyPostPrintProfile(enum telemeterName telemeter, spyPostProfileStruct *profile)
{
	int x = 0;
	int y = 64;

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306FillRect(0, 64 - (SPYPOST_NBITS_SAMPLING_RESOLUTION * 2 - 1), SPYPOST_FIRST_CAL_DISTANCE, SPYPOST_NBITS_SAMPLING_RESOLUTION * 2);

	if (telemeter != TELEMETER_DR && telemeter != TELEMETER_DL)
		return;

	if (telemeter == TELEMETER_DL)
	{
		for (int i = 0; i < SPYPOST_FIRST_CAL_DISTANCE; i++)
		{
			for (int j = SPYPOST_NBITS_SAMPLING_RESOLUTION; j > 0; j--)
			{
				if ((profile->left[i % SPYPOST_FIRST_CAL_DISTANCE] >> (j)) & 0x01)
				{
					ssd1306ClearPixel((100 * i)/80 + x, y - (j*2));
					ssd1306ClearPixel((100 * i)/80 + x, y - (j*2) - 1);
				}
			}
		}
	}
	else
	{
	}
	ssd1306Refresh();
}


//
//	// filter the measure
//	kalman_filter_array(front_telemeters.left, TELEMETER_PROFILE_ARRAY_LENGTH);
//	kalman_filter_array(front_telemeters.right, TELEMETER_PROFILE_ARRAY_LENGTH);
//
//	/*
//	 * this four line are for make sure in search for convert in millimeter we don't go outside
//	 * of the array because we have stronger or smaller value than during the calibration
//	 */
//	front_telemeters.left[0]  = 4095;
//	front_telemeters.right[0] = 4095;
//	front_telemeters.left[TELEMETER_PROFILE_ARRAY_LENGTH]  = 0;
//	front_telemeters.right[TELEMETER_PROFILE_ARRAY_LENGTH] = 0;
//
//	// save the measures
//	bluetoothPrintf("\nfiltered measures :\n");
//	for (int i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; i++)
//	{
//		bluetoothPrintf("%d|%d|%d\n", i,
//				front_telemeters.left[i], front_telemeters.right[i]);
//	}
//	bluetoothPrintf("Saving Front telemeters profile into Flash memory...\n");
//	ssd1306ClearScreen(MAIN_AREA);
//	ssd1306PrintfAtLine(0, 1, &Font_5x8, "save into Flash memory...");
//	// Write telemeters profiles in Flash memory
//	rv = flash_write(zhonxSettings.h_flash,
//			(unsigned char *)&telemeters_profile->front,
//			(unsigned char *)&front_telemeters,
//			sizeof(FRONT_TELEMETERS_PROFILE));
//	if (rv == FLASH_E_SUCCESS)
//	{
//		bluetoothPrintf("Values saved into Flash Memory\n");
//		ssd1306ClearScreen(MAIN_AREA);
//		ssd1306PrintfAtLine(0, 1, &Font_5x8, "FLASH memory updated");
//		ssd1306Refresh();
//	}
//	else
//	{
//		bluetoothPrintf("Failed to write Flash Memory (%d)\n", rv);
//		ssd1306ClearScreen(MAIN_AREA);
//		ssd1306PrintfAtLine(0, 1, &Font_5x8, "FLASH write error (%d)", rv);
//		ssd1306Refresh();
//	}
//	HAL_Delay(3000);
