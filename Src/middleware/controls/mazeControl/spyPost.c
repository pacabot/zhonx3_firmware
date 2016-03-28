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
#define SPYPOST_CAL_DISTANCE			          (int)CELL_LENGTH
#define SPYPOST_ARRAY_PROFILE_LENGTH 		    ((SPYPOST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	32
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	  60
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	  124

#define SPYPOST_REFERENCE_SAMPLE_HEIGHT 	  16	//16 bit height
#define SPYPOST_REFERENCE_SAMPLE_WIDTH 		  20	//array length

#define SPYPOST_MOVE_SPEED 					        50

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM 	((SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_NBITS_SAMPLING_RESOLUTION)

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

// Machine Definitions
typedef struct
{
    int current_sample[SPYPOST_ARRAY_PROFILE_LENGTH + 1];
    int ref_sample[SPYPOST_REFERENCE_SAMPLE_WIDTH + 1];
    enum telemeterName telemeterName;
    int center_distance;
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
static double spyPostGetOffset(spyPostProfileStruct *profile);
static void spyPostStartMeasure(spyPostProfileStruct *profile);
static void spyPostPrintProfile(int x, int y, spyPostProfileStruct *profile);
static void spyPostSendBTProfile(uint32_t *buf32, int lenght_buf32);
static void spyPostKeepUsefulPart(spyPostProfileStruct *profile);
static void spyPostSaveCurrentSampleAsRefSample(spyPostProfileStruct *profile);
static void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width);

double spyPostGetOffset(spyPostProfileStruct *profile)
{
    int i;
    int sample = 0;

    //take the measures
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i)))
               && (hasMoveEnded() != TRUE))
        {
        }
        sample = (int) getTelemeterDist(profile->telemeterName);
        if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            profile->current_sample[i] = 0x00;
        else
            profile->current_sample[i] = 1 << ((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
            SPYPOST_TELEMETER_STEPS_MEASURE_MM);
    }
    spyPostKeepUsefulPart(profile);
    while (hasMoveEnded() != TRUE)
    {
    }
    return 0.0;
}

int spyPostCalibration(void)
{
    spyPostTypeProfileStruct *typeProfile = null;
    spyPostProfileStruct *profile;

    //init stucts
    memset((spyPostTypeProfileStruct*) &left, 0, sizeof(spyPostTypeProfileStruct));
    memset((spyPostTypeProfileStruct*) &right, 0, sizeof(spyPostTypeProfileStruct));
    left.wallToNoWall.telemeterName = TELEMETER_DL;
    left.singlePlot.telemeterName = TELEMETER_DL;
    left.perpendicularWall.telemeterName = TELEMETER_DL;
    right.wallToNoWall.telemeterName = TELEMETER_DR;
    right.singlePlot.telemeterName = TELEMETER_DR;
    right.perpendicularWall.telemeterName = TELEMETER_DR;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (expanderJoyState() != JOY_RIGHT || typeProfile == null)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(spyPostRight, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            typeProfile = &right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(spyPostLeft, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            typeProfile = &left;
        }
        if (expanderJoyFiltered() == JOY_LEFT)
            return SPYPOST_DRIVER_E_SUCCESS;
    }

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(40, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    mainControlInit();
    mainControlSetFollowType(NO_FOLLOW);
    HAL_Delay(4000);
    spyPostStartMeasure(&typeProfile->wallToNoWall);
    spyPostStartMeasure(&typeProfile->singlePlot);
    spyPostStartMeasure(&typeProfile->perpendicularWall);

    for (int i = 0; i < 3; i++)
    {
        switch (i)
        {
            case 0:
                profile = &typeProfile->wallToNoWall;
                break;
            case 1:
                if (typeProfile == &left)
                    profile = &typeProfile->singlePlot;
                else
                    profile = &typeProfile->perpendicularWall;
                break;
            case 2:
                if (typeProfile == &left)
                    profile = &typeProfile->perpendicularWall;
                else
                    profile = &typeProfile->singlePlot;
                break;
        }
        spyPostKeepUsefulPart(profile);
        spyPostSaveCurrentSampleAsRefSample(profile);
        spyPostSampleThicken(profile, 3);
    }
    spyPostPrintProfile(0, 64, &typeProfile->wallToNoWall);
    spyPostPrintProfile(43, 64, &typeProfile->singlePlot);
    spyPostPrintProfile(86, 64, &typeProfile->perpendicularWall);

    while (expanderJoyFiltered() != JOY_LEFT)
    {
    }
    return SPYPOST_DRIVER_E_SUCCESS;
}

void spyPostStartMeasure(spyPostProfileStruct *profile)
{
    //sanity check
    if (profile->telemeterName != TELEMETER_DR && profile->telemeterName != TELEMETER_DL)
        return;

    int i = 0;
    int sample = 0;
    telemetersStart();

    //take the measures
    move(0, SPYPOST_CAL_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i)))
               && (hasMoveEnded() != TRUE))
            ;

        sample = (int) getTelemeterDist(profile->telemeterName);
        if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            profile->current_sample[i] = 0x00;
        else
            profile->current_sample[i] = 1 << ((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
            SPYPOST_TELEMETER_STEPS_MEASURE_MM);

        if (i % 2 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", profile->current_sample[i]);
            ssd1306PrintIntAtLine(0, 0, "wall dist :  ", sample, &Font_5x8);
            ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (int) (SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);
            ssd1306ProgressBar(10, 50, (i * 100) / SPYPOST_ARRAY_PROFILE_LENGTH);
            ssd1306Refresh();
        }
    }
    while (hasMoveEnded() != TRUE)
    {
    }
    telemetersStop();
    motorsDriverSleep(ON);

    ssd1306ClearScreen(MAIN_AREA);
}

void spyPostPrintProfile(int x, int y, spyPostProfileStruct *profile)
{
    ssd1306DrawRect(0 + x, y - 2 - (SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2),
    SPYPOST_REFERENCE_SAMPLE_WIDTH * 2 + 2,
                    SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2 + 2);
    ssd1306PrintfAtLine(x, 0, &Font_3x6, "%d\n", profile->center_distance);
    for (int i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        for (int j = SPYPOST_REFERENCE_SAMPLE_HEIGHT; j > 0; j--)
        {
            if ((profile->ref_sample[i] >> (j)) & 0x01)
            {
                ssd1306DrawPixel(2 * i + x + 1, y - 1 - j * 2);
            }
        }
    }
    ssd1306Refresh();
}

void spyPostSendBTProfile(uint32_t *buf32, int lenght_buf32)
{
    int i = 0;
    for (i = 0; i < lenght_buf32; i++)
    {
        HAL_Delay(4);
        bluetoothPrintf("%d\n", (buf32[i]));
    }
}

void spyPostKeepUsefulPart(spyPostProfileStruct *profile)
{
    int i, j;
    int y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
    // search the minimal of the curve
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        for (j = 0; j < SPYPOST_NBITS_SAMPLING_RESOLUTION; j++)
        {
            if (((profile->current_sample[i] >> j) & 0x01) && y_min > j)
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
    int mask = (int) (pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);
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
    x = SPYPOST_ARRAY_PROFILE_LENGTH - x - 1;
    //save the central point distance
    profile->center_distance = ((SPYPOST_ARRAY_PROFILE_LENGTH - x) - (SPYPOST_REFERENCE_SAMPLE_WIDTH / 2))
            * SPYPOST_ENCODERS_STEPS_MEASURE_MM;
    //sanity check
    if ((SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)) < 0)
    {
        ssd1306PrintfAtLine(0, 1, &Font_5x8, " SPYPOST ERROR x < 0");
        return;
    }
    //move useful part of sample at the beginning of the buffer
    memmove(profile->current_sample,
            profile->current_sample + (SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)),
            SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
    //Clear the rest of the buffer after move
    memset(profile->current_sample + SPYPOST_REFERENCE_SAMPLE_WIDTH, 0,
           (SPYPOST_ARRAY_PROFILE_LENGTH - SPYPOST_REFERENCE_SAMPLE_WIDTH) * 4);
}

void spyPostSaveCurrentSampleAsRefSample(spyPostProfileStruct *profile)
{
    //save crop into ref buffer
    memcpy(profile->ref_sample, profile->current_sample,
    SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
}

void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width)
{
    int i;
    stroke_width /= 2;
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        profile->ref_sample[i] |= (uint16_t) ((profile->ref_sample[i] >> stroke_width)
                | (uint16_t) (profile->ref_sample[i] << stroke_width));
    }
}
