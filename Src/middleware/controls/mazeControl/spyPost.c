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
#include "middleware/controls/mazeControl/basicMoves.h"
#include "middleware/display/pictures.h"
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"

//Declarations for this module */
#include "middleware/controls/mazeControl/spyPost.h"

#define SPYPOST_ENCODERS_STEPS_MEASURE_MM 	   1
#define SPYPOST_CAL_DISTANCE			       (int)MAIN_DIST
#define SPYPOST_ARRAY_PROFILE_LENGTH 		   ((SPYPOST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	   32
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	   60
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	   124

#define SPYPOST_REFERENCE_SAMPLE_HEIGHT 	   16	//16 bit height
#define SPYPOST_REFERENCE_SAMPLE_WIDTH 		   20	//array length

#define SPYPOST_MOVE_SPEED 					   50

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM 	((SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_NBITS_SAMPLING_RESOLUTION)

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

// Machine Definitions
typedef struct
{
    int ref_sample[SPYPOST_REFERENCE_SAMPLE_WIDTH + 1];
    int ref_center_distance;
    int stat;
} spyPostProfileStruct;

typedef struct
{
    spyPostProfileStruct wallToNoWall;
    spyPostProfileStruct singlePost;
    spyPostProfileStruct perpendicularWall;
    int current_sample[SPYPOST_ARRAY_PROFILE_LENGTH + 1];
    int current_center_distance;
    enum telemeterName telemeterName;
} spyPostTypeProfileStruct;

spyPostTypeProfileStruct left;
spyPostTypeProfileStruct right;

spyPostTypeProfileStruct left = {
                    .wallToNoWall =
                    {
                     .ref_sample = {14,
                                    28,
                                    28,
                                    28,
                                    28,
                                    28,
                                    28,
                                    56,
                                    56,
                                    56,
                                    56,
                                    112,
                                    224,
                                    224,
                                    448,
                                    896,
                                    1792,
                                    14336,
                                    14336,
                                    49152},
                     .ref_center_distance = 55
                    },
                    .singlePost =
                    {
                     .ref_sample = {448,
                                    224,
                                    112,
                                    56,
                                    28,
                                    14,
                                    14,
                                    14,
                                    7,
                                    7,
                                    14,
                                    14,
                                    28,
                                    56,
                                    112,
                                    448,
                                    896,
                                    3584,
                                    14336,
                                    57344},
                     .ref_center_distance = 60
                    },
                    .perpendicularWall =
                    {
                     .ref_sample = {112,
                                    56,
                                    28,
                                    28,
                                    14,
                                    14,
                                    7,
                                    7,
                                    7,
                                    7,
                                    7,
                                    14,
                                    14,
                                    56,
                                    112,
                                    224,
                                    896,
                                    7168,
                                    28672,
                                    49152},
                     .ref_center_distance = 60
                    }
};

spyPostTypeProfileStruct right = {
                    .wallToNoWall =
                    {
                     .ref_sample = {14,
                                    14,
                                    14,
                                    14,
                                    14,
                                    14,
                                    14,
                                    14,
                                    28,
                                    28,
                                    28,
                                    56,
                                    56,
                                    112,
                                    224,
                                    448,
                                    896,
                                    1792,
                                    7168,
                                    28672},
                     .ref_center_distance = 56
                    },
                    .singlePost =
                    {
                     .ref_sample = {7168,
                                    1792,
                                    896,
                                    448,
                                    112,
                                    56,
                                    28,
                                    14,
                                    14,
                                    14,
                                    7,
                                    7,
                                    14,
                                    14,
                                    28,
                                    112,
                                    224,
                                    896,
                                    3584,
                                    14336},
                     .ref_center_distance = 59
                    },
                    .perpendicularWall =
                    {
                     .ref_sample = {448,
                                    112,
                                    56,
                                    28,
                                    28,
                                    14,
                                    7,
                                    7,
                                    7,
                                    7,
                                    7,
                                    7,
                                    14,
                                    28,
                                    56,
                                    224,
                                    448,
                                    1792,
                                    7168,
                                    28672},
                     .ref_center_distance = 60
                    },
};

// Declare telemeters profiles in Flash memory
//TELEMETERS_PROFILE *telemeters_profile = (TELEMETERS_PROFILE *) ADDR_FLASH_SECTOR_10;

/* Static functions */
static void spyPostStartMeasure(spyPostTypeProfileStruct *typeProfile);
static void spyPostPrintProfile(int x, int y, spyPostProfileStruct *profile);
static void spyPostSendBTProfile(int *buf32, int lenght_buf32);
static void spyPostKeepUsefulPart(spyPostTypeProfileStruct *typeProfile);
static void spyPostSaveCurrentSampleAsRefSample(spyPostTypeProfileStruct *typeProfile, spyPostProfileStruct *profile);
static void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width);

/*
 *
 *      o         o
 *      :    |    :
 *      :    |    :
 *      o         o
 */
int spyPostGetOffset(spyPostGetOffsetsStruct *offset)
{
    int i;
    int left_sample = 0;
    int right_sample = 0;

    //clear stats
    left.wallToNoWall.stat = 0;
    left.singlePost.stat = 0;
    left.perpendicularWall.stat = 0;
    right.wallToNoWall.stat = 0;
    right.singlePost.stat = 0;
    right.perpendicularWall.stat = 0;

    offset->left_x = 0;
    offset->right_x = 0;

    //take the measures
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i)))
               && (hasMoveEnded() != TRUE))
        {
        }
        left_sample = (int) getTelemeterDist(TELEMETER_DL);
        right_sample = (int) getTelemeterDist(TELEMETER_DR);
        // left statement
        if (left_sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || left_sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            left.current_sample[i] = 0x00;
        else
            left.current_sample[i] = 1 << ((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
            SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        // right statement
        if (right_sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || right_sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            right.current_sample[i] = 0x00;
        else
            right.current_sample[i] = 1 << ((right_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
            SPYPOST_TELEMETER_STEPS_MEASURE_MM);
    }
    spyPostKeepUsefulPart(&left);
    spyPostKeepUsefulPart(&right);
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        if (left.current_sample[i] & left.wallToNoWall.ref_sample[i])
        {
            left.wallToNoWall.stat++;
        }
        if (left.current_sample[i] & left.singlePost.ref_sample[i])
        {
            left.singlePost.stat++;
        }
        if (left.current_sample[i] & left.perpendicularWall.ref_sample[i])
        {
            left.perpendicularWall.stat++;
        }
        if (right.current_sample[i] & right.wallToNoWall.ref_sample[i])
        {
            right.wallToNoWall.stat++;
        }
        if (right.current_sample[i] & right.singlePost.ref_sample[i])
        {
            right.singlePost.stat++;
        }
        if (right.current_sample[i] & right.perpendicularWall.ref_sample[i])
        {
            right.perpendicularWall.stat++;
        }
    }

    if (left.wallToNoWall.stat > left.perpendicularWall.stat)
    {
        if (left.wallToNoWall.stat > left.singlePost.stat)
            offset->left_x = left.wallToNoWall.ref_center_distance - left.current_center_distance;
        else
            offset->left_x = left.singlePost.ref_center_distance - left.current_center_distance;
    }
    else if (left.perpendicularWall.stat > left.singlePost.stat)
        offset->left_x = left.perpendicularWall.ref_center_distance - left.current_center_distance;
    else
        offset->left_x = left.singlePost.ref_center_distance - left.current_center_distance;

    if (right.wallToNoWall.stat > right.perpendicularWall.stat)
    {
        if (right.wallToNoWall.stat > right.singlePost.stat)
            offset->right_x = right.wallToNoWall.ref_center_distance - right.current_center_distance;
        else
            offset->right_x = right.singlePost.ref_center_distance - right.current_center_distance;
    }
    else if (right.perpendicularWall.stat > right.singlePost.stat)
        offset->right_x = right.perpendicularWall.ref_center_distance - right.current_center_distance;
    else
        offset->right_x = right.singlePost.ref_center_distance - right.current_center_distance;

    return 0;
}

int spyPostCalibration(void)
{
    spyPostTypeProfileStruct *typeProfile = null;
    spyPostProfileStruct *profile;

    //init stucts
    memset((spyPostTypeProfileStruct*) &left, 0, sizeof(spyPostTypeProfileStruct));
    memset((spyPostTypeProfileStruct*) &right, 0, sizeof(spyPostTypeProfileStruct));
    left.telemeterName = TELEMETER_DL;
    right.telemeterName = TELEMETER_DR;

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

    for (int i = 0; i < 3; i++)
    {
        spyPostStartMeasure(typeProfile);
        switch (i)
        {
            case 0:
                profile = &typeProfile->wallToNoWall;
                break;
            case 1:
                if (typeProfile == &left)
                    profile = &typeProfile->singlePost;
                else
                    profile = &typeProfile->perpendicularWall;
                break;
            case 2:
                if (typeProfile == &left)
                    profile = &typeProfile->perpendicularWall;
                else
                    profile = &typeProfile->singlePost;
                break;
        }
        spyPostKeepUsefulPart(typeProfile);
        spyPostSaveCurrentSampleAsRefSample(typeProfile, profile);
        spyPostSampleThicken(profile, 3);
    }
    spyPostPrintProfile(0, 64, &typeProfile->wallToNoWall);
    spyPostPrintProfile(43, 64, &typeProfile->singlePost);
    spyPostPrintProfile(86, 64, &typeProfile->perpendicularWall);

    //send BT profiles
    bluetoothPrintf("wallToNoWall x = %d\n\r", typeProfile->wallToNoWall.ref_center_distance);
    spyPostSendBTProfile((int*)&typeProfile->wallToNoWall.ref_sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("singlePost x = %d\n\r", typeProfile->singlePost.ref_center_distance);
    spyPostSendBTProfile((int*)&typeProfile->singlePost.ref_sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("perpendicularWall x = %d\n\r", typeProfile->perpendicularWall.ref_center_distance);
    spyPostSendBTProfile((int*)&typeProfile->perpendicularWall.ref_sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);

    // Write spyPost profiles in Flash memory
//    int rv = flash_write(zhonxSettings.h_flash, (unsigned char *) &telemeters_profile->front,
//                     (unsigned char *) &front_telemeters, sizeof(FRONT_TELEMETERS_PROFILE));
//    if (rv == FLASH_E_SUCCESS)
//    {
//        bluetoothPrintf("Values saved into Flash Memory\n");
//        ssd1306ClearScreen(MAIN_AREA);
//        ssd1306PrintfAtLine(0, 1, &Font_5x8, "FLASH memory updated");
//        ssd1306Refresh();
//    }
//    else
//    {
//        bluetoothPrintf("Failed to write Flash Memory (%d)\n", rv);
//        ssd1306ClearScreen(MAIN_AREA);
//        ssd1306PrintfAtLine(0, 1, &Font_5x8, "FLASH write error (%d)", rv);
//        ssd1306Refresh();
//    }
//    HAL_Delay(3000);

    while (expanderJoyFiltered() != JOY_LEFT)
    {
    }
    return SPYPOST_DRIVER_E_SUCCESS;
}

/*
 *
 *      o    :    o
 *      :    |    :
 *      :    |    :
 *      o    :    o
 */
void spyPostStartMeasure(spyPostTypeProfileStruct *typeProfile)
{
    //sanity check
    if (typeProfile->telemeterName != TELEMETER_DR && typeProfile->telemeterName != TELEMETER_DL)
        return;

    int i = 0;
    int sample = 0;
    telemetersStart();

    //offset dist
    move(0, OFFSET_DIST, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    while (hasMoveEnded() != TRUE)
    {
    }
    //take the measures
    move(0, SPYPOST_CAL_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i)))
               && (hasMoveEnded() != TRUE))
            ;

        sample = (int) getTelemeterDist(typeProfile->telemeterName);
        if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            typeProfile->current_sample[i] = 0x00;
        else
            typeProfile->current_sample[i] = 1 << ((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
            SPYPOST_TELEMETER_STEPS_MEASURE_MM);

        if (i % 2 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", typeProfile->current_sample[i]);
            ssd1306PrintIntAtLine(0, 0, "wall dist :  ", sample, &Font_5x8);
            ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (int) (SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);
            ssd1306ProgressBar(10, 50, (i * 100) / SPYPOST_ARRAY_PROFILE_LENGTH);
            ssd1306Refresh();
        }
    }
    while (hasMoveEnded() != TRUE)
    {
    }
    //offset dist
    move(0, OFFSET_DIST, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
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
    ssd1306PrintfAtLine(x, 1, &Font_3x6, "%d\n", profile->ref_center_distance);
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

void spyPostSendBTProfile(int *buf32, int lenght_buf32)
{
    int i = 0;
    for (i = 0; i < lenght_buf32; i++)
    {
        bluetoothWaitReady();
        bluetoothPrintf("%d\n", (buf32[i]));
    }
    bluetoothWaitReady();
}

void spyPostKeepUsefulPart(spyPostTypeProfileStruct *typeProfile)
{
    int i, j;
    int y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
    // search the minimal of the curve
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        for (j = 0; j < SPYPOST_NBITS_SAMPLING_RESOLUTION; j++)
        {
            if (((typeProfile->current_sample[i] >> j) & 0x01) && y_min > j)
            {
                y_min = j;
            }
        }
    }
    //shift buffer (y axis)
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        typeProfile->current_sample[i] = typeProfile->current_sample[i] >> (y_min - 1); //-& for one pix margin
    }
    //crop buffer height
    int mask = (int) (pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        typeProfile->current_sample[i] = typeProfile->current_sample[i] & mask;
    }
    //search the last value into the buffer for align reference sample
    int x = SPYPOST_ARRAY_PROFILE_LENGTH;
    while (typeProfile->current_sample[x] == 0x00)
    {
        x--;
    }
    x = SPYPOST_ARRAY_PROFILE_LENGTH - x - 1;
    //save the central point distance
    typeProfile->current_center_distance = ((SPYPOST_ARRAY_PROFILE_LENGTH - x) - (SPYPOST_REFERENCE_SAMPLE_WIDTH / 2))
            * SPYPOST_ENCODERS_STEPS_MEASURE_MM;
    //sanity check
    if ((SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)) < 0)
    {
        ssd1306PrintfAtLine(0, 0, &Font_5x8, " SPYPOST ERROR x < 0");
        return;
    }
    //move useful part of sample at the beginning of the buffer
    memmove(typeProfile->current_sample,
            typeProfile->current_sample + (SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)),
            SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
    //Clear the rest of the buffer after move
    memset(typeProfile->current_sample + SPYPOST_REFERENCE_SAMPLE_WIDTH, 0,
           (SPYPOST_ARRAY_PROFILE_LENGTH - SPYPOST_REFERENCE_SAMPLE_WIDTH) * 4);
}

void spyPostSaveCurrentSampleAsRefSample(spyPostTypeProfileStruct *typeProfile, spyPostProfileStruct *profile)
{
    //save crop into ref buffer
    memcpy(profile->ref_sample, typeProfile->current_sample,
    SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
    profile->ref_center_distance = typeProfile->current_center_distance;
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

int spyPostReadCalibration(void)
{
    spyPostTypeProfileStruct *typeProfile = null;
    spyPostProfileStruct *profile;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (expanderJoyState() != JOY_RIGHT || typeProfile == null)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            typeProfile = &right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            typeProfile = &left;
        }
        if (expanderJoyFiltered() == JOY_LEFT)
            return SPYPOST_DRIVER_E_SUCCESS;
    }

    spyPostPrintProfile(0, 64, &typeProfile->wallToNoWall);
    spyPostPrintProfile(43, 64, &typeProfile->singlePost);
    spyPostPrintProfile(86, 64, &typeProfile->perpendicularWall);

    while (expanderJoyFiltered() != JOY_LEFT)
    {
    }
    return SPYPOST_DRIVER_E_SUCCESS;
}

void spyPostTest()
{
    spyPostGetOffsetsStruct offset;

    ssd1306ClearScreen(MAIN_AREA);
    mainControlInit();
    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);

    HAL_Delay(2000);

    int Vmin, Vmax, Vrotate;
    Vmin = 100;
    Vmax = 100;

    moveStartCell(Vmax, Vmax);
    while (hasMoveEnded() != TRUE)
    {
    }
    move(0, MAIN_DIST, Vmax, Vmax); //distance with last move offset
    spyPostGetOffset(&offset);
    while (hasMoveEnded() != TRUE)
    {
    }
    if (offset.left_x != 0)
    {
        move(0, (OFFSET_DIST * 2.00) + offset.left_x, Vmax, Vmax);
        ssd1306PrintfAtLine(0, 2, &Font_5x8, "left_x = %d", offset.left_x);
    }
    else
    {
        move(0, (OFFSET_DIST * 2.00) + offset.right_x, Vmax, Vmax);
        ssd1306PrintfAtLine(0, 2, &Font_5x8, "right_x = %d", offset.right_x);
    }

    ssd1306Refresh();
    telemetersStop();
    HAL_Delay(5000);
    motorsDriverSleep(ON);
}
