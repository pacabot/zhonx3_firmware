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
#define SPYPOST_OFFSET_DISTANCE				10
#define SPYPOST_FIRST_CAL_DISTANCE			40
#define SPYPOST_ARRAY_PROFILE_LENGTH 		((SPYPOST_FIRST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	32
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	70
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	102

#define SPYPOST_REFERENCE_SAMPLE_HEIGHT 	16	//16 bit height
#define SPYPOST_REFERENCE_SAMPLE_WIDTH 		20	//array length

#define SPYPOST_MOVE_SPEED 					50

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM 	((SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_NBITS_SAMPLING_RESOLUTION)

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

// Machine Definitions
typedef struct
{
	uint32_t current_sample [SPYPOST_ARRAY_PROFILE_LENGTH];
	uint16_t ref_sample [SPYPOST_REFERENCE_SAMPLE_WIDTH];
	enum telemeterName telemeterName;
} spyPostProfileStruct;

typedef struct
{
	spyPostProfileStruct wallToNoWall;
	spyPostProfileStruct singlePlot;
	spyPostProfileStruct perpendicularWall;
} spyPostTypeProfileStruct;

spyPostTypeProfileStruct left;
spyPostTypeProfileStruct right;

/* Static functions */
static void spyPostStartMeasure(spyPostProfileStruct *profile);
static void spyPostPrintProfile(spyPostProfileStruct *profile);
static void spyPostSendBTProfile(int *buff, int lenght);
static void spyPostResizeCalibrationProfile(spyPostProfileStruct *profile);

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
	spyPostProfileStruct spyPostProfile_singlePlot;
	spyPostProfileStruct spyPostProfile_perpendicularWall;

	//init stucts
	memset((spyPostTypeProfileStruct*)&left, 0, sizeof(spyPostTypeProfileStruct));
	memset((spyPostTypeProfileStruct*)&right, 0, sizeof(spyPostTypeProfileStruct));
	left.wallToNoWall.telemeterName = TELEMETER_DL;
	left.singlePlot.telemeterName = TELEMETER_DL;
	left.perpendicularWall.telemeterName = TELEMETER_DL;
	right.wallToNoWall.telemeterName = TELEMETER_DR;
	right.singlePlot.telemeterName = TELEMETER_DR;
	right.perpendicularWall.telemeterName = TELEMETER_DR;

	//left calibration
	spyPostStartMeasure(&left.wallToNoWall);
	spyPostStartMeasure(&left.singlePlot);
	spyPostStartMeasure(&left.perpendicularWall);

	spyPostResizeCalibrationProfile(&left.wallToNoWall);

	while(1);
	return 0;
	//	return TELEMETERS_DRIVER_E_SUCCESS;
}

void spyPostStartMeasure(spyPostProfileStruct *profile)
{
	int buff[SPYPOST_ARRAY_PROFILE_LENGTH] = {0};

	if (profile->telemeterName != TELEMETER_DR && profile->telemeterName != TELEMETER_DL)
		return;

	int i = 0;
	int sample = 0;
	telemetersStart();

	//offset dist
	move(0, SPYPOST_OFFSET_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
	while(hasMoveEnded() != TRUE);
	//take the measures
	move(0, SPYPOST_FIRST_CAL_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
	for(i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
	{
		while((((int)(encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) &&
				(hasMoveEnded() != TRUE));

		ssd1306ClearScreen(MAIN_AREA);
		sample = (int)getTelemeterDist(profile->telemeterName);
		if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
			profile->current_sample[i] = 0x00;
		else
			profile->current_sample[i] = 1 << (sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM;
		ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", profile->current_sample[i]);

		buff[i] = (sample * 10);
		ssd1306PrintIntAtLine(0, 0, "wall dist :  ", sample, &Font_5x8);
		ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (int)(SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);

		ssd1306ProgressBar(10,50,(i*100)/SPYPOST_ARRAY_PROFILE_LENGTH);
		ssd1306Refresh();
	}

	while(hasMoveEnded() != TRUE);
	move(0, CELL_LENGTH - (SPYPOST_FIRST_CAL_DISTANCE + SPYPOST_OFFSET_DISTANCE), SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED); //
	while(hasMoveEnded() != TRUE);
	telemetersStop();
	motorsDriverSleep(ON);

	//spyPostSendBTProfile(buff, SPYPOST_ARRAY_PROFILE_LENGTH);
}

void spyPostPrintProfile(spyPostProfileStruct *profile)
{
	int x = 0;
	int y = 64;

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306DrawRect(0, 64 - (SPYPOST_REFERENCE_SAMPLE_HEIGHT * 4 + 1), SPYPOST_REFERENCE_SAMPLE_WIDTH * 5, SPYPOST_REFERENCE_SAMPLE_HEIGHT * 4);

	for (int i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
	{
		for (int j = SPYPOST_REFERENCE_SAMPLE_HEIGHT; j > 0; j--)
		{
			if ((profile->ref_sample[i] >> (j)) & 0x01)
			{
				ssd1306DrawPixel(5 * i + x, y - j * 4);
			}
		}
	}

	ssd1306Refresh();
}

void spyPostSendBTProfile(int *buff, int lenght)
{
	int i = 0;
	for (i = 0; i < lenght; i++)
	{
		HAL_Delay(2);
		bluetoothPrintf("%d\n", (buff[i]));
	}
}

void spyPostResizeCalibrationProfile(spyPostProfileStruct *profile)
{
	int i, j;
	int y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
	// search the minimal of the curve
	for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
	{
		for (j = 0; j < SPYPOST_NBITS_SAMPLING_RESOLUTION; j++)
		{
			if (((profile->current_sample[i] >> j) & 0x01) && y_min > j )
			{
				y_min = j;
			}
		}
	}
	//shift buffer (y axis)
	for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
	{
		profile->current_sample[i] = profile->current_sample[i] >> (y_min - 1); //-& for one pix margin
	}
	//crop buffer height
	int mask = (int)(pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);
	for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
	{
		profile->current_sample[i] = profile->current_sample[i] & mask;
	}
	//search the last value into the buffer for align reference sample
	int x = SPYPOST_ARRAY_PROFILE_LENGTH;
	while (profile->current_sample[x] == 0x00)
	{
		x--;
	}
	//save crop into ref buffer
	for (i = SPYPOST_REFERENCE_SAMPLE_WIDTH; i > 0; i++)
	{
		profile->ref_sample[i] = (uint16_t)profile->current_sample[i - x];
	}
}
