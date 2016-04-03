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
    int sample[SPYPOST_REFERENCE_SAMPLE_WIDTH + 1];
    int center_x_distance;
    int center_y_distance;
} spyPostProfileStruct;

typedef struct
{
    spyPostProfileStruct wallToNoWall;
    spyPostProfileStruct singlePost;
    spyPostProfileStruct perpendicularWall;
} spyPostRefTypeProfileStruct;

spyPostRefTypeProfileStruct ref_left = //todo add saved value into flash
{ .wallToNoWall =
{ .sample =
{ 14, 28, 28, 28, 28, 28, 28, 56, 56, 56, 56, 112, 224, 224, 448, 896, 1792, 14336, 14336, 49152 }, .center_x_distance =
        55 }, .singlePost =
        { .sample =
        { 448, 224, 112, 56, 28, 14, 14, 14, 7, 7, 14, 14, 28, 56, 112, 448, 896, 3584, 14336, 57344 }, .center_x_distance =
                60 }, .perpendicularWall =
                { .sample =
                { 112, 56, 28, 28, 14, 14, 7, 7, 7, 7, 7, 14, 14, 56, 112, 224, 896, 7168, 28672, 49152 }, .center_x_distance =
                        60 } };

spyPostRefTypeProfileStruct ref_right = //todo add saved value into flash
{ .wallToNoWall =
{ .sample =
{ 14, 14, 14, 14, 14, 14, 14, 14, 28, 28, 28, 56, 56, 112, 224, 448, 896, 1792, 7168, 28672 }, .center_x_distance =
        56 }, .singlePost =
        { .sample =
        { 7168, 1792, 896, 448, 112, 56, 28, 14, 14, 14, 7, 7, 14, 14, 28, 112, 224, 896, 3584, 14336 }, .center_x_distance =
                59 }, .perpendicularWall =
                { .sample = { 448, 112, 56, 28, 28, 14, 7, 7, 7, 7, 7, 7, 14, 28, 56, 224, 448, 1792, 7168, 28672 }, .center_x_distance =
                        60 }, };

// Declare telemeters profiles in Flash memory
//TELEMETERS_PROFILE *telemeters_profile = (TELEMETERS_PROFILE *) ADDR_FLASH_SECTOR_10;

/* Static functions */
static void spyPostStartMeasure(spyPostProfileStruct *currentProfile, enum telemeterName telemeterName);
static void spyPostPrintProfile(int x, int y, int *sample, int center_x_distance);
static void spyPostSendBTProfile(int *buf32, int lenght_buf32);
static void spyPostKeepUsefulPart(spyPostProfileStruct *typeProfile);
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
    spyPostProfileStruct current_left;
    spyPostProfileStruct current_right;

    //clear stats
    int left_wallToNoWall_stat = 0;
    int left_singlePost_stat = 0;
    int left_perpendicularWall_stat = 0;
    int right_wallToNoWall_stat = 0;
    int right_singlePost_stat = 0;
    int right_perpendicularWall_stat = 0;

    memset(&current_right.sample, 0, sizeof(current_right.sample));
    memset(&current_left.sample, 0, sizeof(current_left.sample));

    offset->left_x = 0;
    offset->right_x = 0;

    //take the measures
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) && (hasMoveEnded() != TRUE))
        {
        }
        left_sample = (int) getTelemeterDist(TELEMETER_DL);
        right_sample = (int) getTelemeterDist(TELEMETER_DR);
        // left statement
        if (left_sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || left_sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            current_left.sample[i] = 0x00;
        else
            current_left.sample[i] = 1 << ((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        // right statement
        if (right_sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || right_sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            current_right.sample[i] = 0x00;
        else
            current_right.sample[i] = 1 << ((right_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
    }
    spyPostKeepUsefulPart(&current_left);
    spyPostKeepUsefulPart(&current_right);
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        if (current_left.sample[i] & ref_left.wallToNoWall.sample[i])
        {
            left_wallToNoWall_stat++;
        }
        if (current_left.sample[i] & ref_left.singlePost.sample[i])
        {
            left_singlePost_stat++;
        }
        if (current_left.sample[i] & ref_left.perpendicularWall.sample[i])
        {
            left_perpendicularWall_stat++;
        }

        if (current_right.sample[i] & ref_right.wallToNoWall.sample[i])
        {
            right_wallToNoWall_stat++;
        }
        if (current_right.sample[i] & ref_right.singlePost.sample[i])
        {
            right_singlePost_stat++;
        }
        if (current_right.sample[i] & ref_right.perpendicularWall.sample[i])
        {
            right_perpendicularWall_stat++;
        }
    }

    if (left_wallToNoWall_stat < 15)
        left_wallToNoWall_stat = 0;
    if (left_singlePost_stat < 15)
        left_singlePost_stat = 0;
    if (left_perpendicularWall_stat < 15)
        left_perpendicularWall_stat = 0;
    if (right_wallToNoWall_stat < 15)
        right_wallToNoWall_stat = 0;
    if (right_singlePost_stat < 15)
        right_singlePost_stat = 0;
    if (right_perpendicularWall_stat < 15)
        right_perpendicularWall_stat = 0;

    if (left_wallToNoWall_stat > left_perpendicularWall_stat)
    {
        if (left_wallToNoWall_stat > left_singlePost_stat)
        {
            offset->left_x = ref_left.wallToNoWall.center_x_distance - current_left.center_x_distance;
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("L_WTNW = %d, stat = %d\n", (int) offset->left_x, (int) left_wallToNoWall_stat);
#endif
        }
        else
        {
            offset->left_x = ref_left.singlePost.center_x_distance - current_left.center_x_distance;
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("L_SP = %d, stat = %d\n", (int) offset->left_x, (int) left_singlePost_stat);
#endif
        }
    }
    else if (left_perpendicularWall_stat > left_singlePost_stat)
    {
        offset->left_x = ref_left.perpendicularWall.center_x_distance - current_left.center_x_distance;
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("L_PW = %d, stat = %d\n", (int) offset->left_x, (int) left_perpendicularWall_stat);
#endif
    }
    else if (left_singlePost_stat != 0)
    {
        offset->left_x = ref_left.singlePost.center_x_distance - current_left.center_x_distance;
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("L_SP = %d, stat = %d\n", (int) offset->left_x, (int) left_singlePost_stat);
#endif
    }
    else
        offset->left_x = 0;

    /*--------------------------------------------------------------------------------------------------------------------*/
    if (right_wallToNoWall_stat > right_perpendicularWall_stat)
    {
        if (right_wallToNoWall_stat > right_singlePost_stat)
        {
            offset->right_x = ref_right.wallToNoWall.center_x_distance - current_right.center_x_distance;
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("R_WTNW = %d, stat = %d\n", (int) offset->right_x, (int) right_wallToNoWall_stat);
#endif
        }
        else
        {
            offset->right_x = ref_right.singlePost.center_x_distance - current_right.center_x_distance;
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("R_SP = %d, stat = %d\n", (int) offset->right_x, (int) right_singlePost_stat);
#endif
        }
    }
    else if (right_perpendicularWall_stat > right_singlePost_stat)
    {
        offset->right_x = ref_right.perpendicularWall.center_x_distance - current_right.center_x_distance;
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("R_PW = %d, stat = %d\n", (int) offset->right_x, (int) right_perpendicularWall_stat);
#endif
    }
    else if (right_singlePost_stat != 0)
    {
        offset->right_x = ref_right.singlePost.center_x_distance - current_right.center_x_distance;
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("R_SP = %d, stat = %d\n", (int) offset->right_x, (int) right_singlePost_stat);
#endif
    }
    else
        offset->right_x = 0;

    return SPYPOST_DRIVER_E_SUCCESS;
}

int spyPostCalibration(void)
{
    spyPostRefTypeProfileStruct *refProfiles = null;
    spyPostProfileStruct currentProfile;

    //init stucts
    memset((spyPostRefTypeProfileStruct*) &ref_left, 0, sizeof(spyPostRefTypeProfileStruct));
    memset((spyPostRefTypeProfileStruct*) &ref_right, 0, sizeof(spyPostRefTypeProfileStruct));

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (expanderJoyState() != JOY_RIGHT || refProfiles == null)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(spyPostRight, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = &ref_right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(spyPostLeft, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = &ref_left;
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
        if (refProfiles == &ref_left)
            spyPostStartMeasure(&currentProfile, TELEMETER_DL);
        else
            spyPostStartMeasure(&currentProfile, TELEMETER_DR);

        spyPostKeepUsefulPart(&currentProfile);
        spyPostSampleThicken(&currentProfile, 3);
        switch (i)
        {
            case 0:
                refProfiles->wallToNoWall = currentProfile; //todo add saved value into flash
                break;
            case 1:
                if (refProfiles == &ref_left)
                    refProfiles->singlePost = currentProfile; //todo add saved value into flash
                else
                    refProfiles->perpendicularWall = currentProfile; //todo add saved value into flash
                break;
            case 2:
                if (refProfiles == &ref_left)
                    refProfiles->perpendicularWall = currentProfile; //todo add saved value into flash
                else
                    refProfiles->singlePost = currentProfile; //todo add saved value into flash
                break;
        }
    }
    spyPostPrintProfile(0, 64, refProfiles->wallToNoWall.sample, refProfiles->wallToNoWall.center_x_distance);
    spyPostPrintProfile(43, 64, refProfiles->singlePost.sample, refProfiles->singlePost.center_x_distance);
    spyPostPrintProfile(86, 64, refProfiles->perpendicularWall.sample, refProfiles->perpendicularWall.center_x_distance);

    //send BT profiles
    bluetoothPrintf("wallToNoWall x = %d\n\r", refProfiles->wallToNoWall.center_x_distance);
    spyPostSendBTProfile((int*)&refProfiles->wallToNoWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("singlePost x = %d\n\r", refProfiles->singlePost.center_x_distance);
    spyPostSendBTProfile((int*) &refProfiles->singlePost.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("perpendicularWall x = %d\n\r", refProfiles->perpendicularWall.center_x_distance);
    spyPostSendBTProfile((int*) &refProfiles->perpendicularWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);

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
void spyPostStartMeasure(spyPostProfileStruct *currentProfile, enum telemeterName telemeterName)
{
    //sanity check
    if (telemeterName != TELEMETER_DR && telemeterName != TELEMETER_DL)
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
        while ((((int)(encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) && (hasMoveEnded() != TRUE))
            ;

        sample = (int)getTelemeterDist(telemeterName);
        if (sample < SPYPOST_MIN_DIAG_SENSOR_DISTANCE || sample > SPYPOST_MAX_DIAG_SENSOR_DISTANCE)
            currentProfile->sample[i] = 0x00;
        else
            currentProfile->sample[i] = 1 << ((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);

        if (i % 2 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", (int)currentProfile->sample[i]);
            ssd1306PrintIntAtLine(0, 0, "wall dist :  ", (int)sample, &Font_5x8);
            ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (int)(SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);
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

void spyPostPrintProfile(int x, int y, int *sample, int center_x_distance)
{
    ssd1306DrawRect(0 + x, y - 2 - (SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2),
                    SPYPOST_REFERENCE_SAMPLE_WIDTH * 2 + 2,
                    SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2 + 2);
    ssd1306PrintfAtLine(x, 1, &Font_3x6, "%d\n", center_x_distance);
    for (int i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        for (int j = SPYPOST_REFERENCE_SAMPLE_HEIGHT; j > 0; j--)
        {
            if ((sample[i] >> (j)) & 0x01)
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

void spyPostKeepUsefulPart(spyPostProfileStruct *currentProfile)
{
    int i, j;
    int y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
    // search the minimal of the curve
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        for (j = 0; j < SPYPOST_NBITS_SAMPLING_RESOLUTION; j++)
        {
            if (((currentProfile->sample[i] >> j) & 0x01) && y_min > j)
            {
                y_min = j;
            }
        }
    }
    //shift buffer (y axis)
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] = currentProfile->sample[i] >> (y_min - 1); // for one pix margin
    }
    //crop buffer height
    int mask = (int) (pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] = currentProfile->sample[i] & mask;
    }
    //search the last value into the buffer for align reference sample
    int x = SPYPOST_ARRAY_PROFILE_LENGTH;
    while (currentProfile->sample[x] == 0x00)
    {
        x--;
    }
    x = SPYPOST_ARRAY_PROFILE_LENGTH - x - 1;
    //save the central point distance
    currentProfile->center_x_distance = ((SPYPOST_ARRAY_PROFILE_LENGTH - x) - (SPYPOST_REFERENCE_SAMPLE_WIDTH / 2))
                    * SPYPOST_ENCODERS_STEPS_MEASURE_MM;
    //sanity check
    if ((SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)) < 0)
    {
        //        memset(&typeProfile->sample, 0, sizeof(typeProfile->sample));
        ssd1306PrintfAtLine(0, 0, &Font_5x8, " SPYPOST ERROR x < 0");
        return;
    }
    //move useful part of sample at the beginning of the buffer
    memmove(currentProfile->sample,
            currentProfile->sample + (SPYPOST_ARRAY_PROFILE_LENGTH - (x + SPYPOST_REFERENCE_SAMPLE_WIDTH)),
            SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
    //Clear the rest of the buffer after move
    memset(currentProfile->sample + SPYPOST_REFERENCE_SAMPLE_WIDTH, 0,
           (SPYPOST_ARRAY_PROFILE_LENGTH - SPYPOST_REFERENCE_SAMPLE_WIDTH) * 4);
}

void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width)
{
    int i;
    stroke_width /= 2;
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        profile->sample[i] |= (uint16_t) ((profile->sample[i] >> stroke_width)
                | (uint16_t) (profile->sample[i] << stroke_width));
    }
}

int spyPostReadCalibration(void)
{
    spyPostRefTypeProfileStruct *refProfiles = null;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (expanderJoyState() != JOY_RIGHT || refProfiles == null)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = &ref_right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = &ref_left;
        }
        if (expanderJoyFiltered() == JOY_LEFT)
            return SPYPOST_DRIVER_E_SUCCESS;
    }
    spyPostPrintProfile(0, 64, refProfiles->wallToNoWall.sample, refProfiles->wallToNoWall.center_x_distance);
    spyPostPrintProfile(43, 64, refProfiles->singlePost.sample, refProfiles->singlePost.center_x_distance);
    spyPostPrintProfile(86, 64, refProfiles->perpendicularWall.sample, refProfiles->perpendicularWall.center_x_distance);

    while (expanderJoyFiltered() != JOY_LEFT)
    {
    }
    return SPYPOST_DRIVER_E_SUCCESS;
}

void spyPostTest()
{
    spyPostGetOffsetsStruct offset;
    memset((spyPostGetOffsetsStruct*) &offset, 0, sizeof(spyPostGetOffsetsStruct));

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
        move(0, (OFFSET_DIST * 2.00) - offset.left_x, Vmax, Vmax);
        ssd1306PrintfAtLine(70, 1, &Font_3x6, "L_x = %d", offset.left_x);
    }
    else
    {
        move(0, (OFFSET_DIST * 2.00) - offset.right_x, Vmax, Vmax);
        ssd1306PrintfAtLine(70, 1, &Font_3x6, "R_x = %d", offset.right_x);
    }

    ssd1306Refresh();
    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT)
    {
    }
}
