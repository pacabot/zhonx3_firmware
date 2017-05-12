/**************************************************************************/
/*!
 @file      spyPost.c
 @author    PLF (PACABOT)
 @date      Created on: 19 mars 2016
 @version   1.7
 */
/**************************************************************************/
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
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/eeprom/24lc64.h"
#include "peripherals/flash/flash.h"

/* Middleware declarations */
#include "middleware/display/pictures.h"
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/moves/mazeMoves/mazeMoves.h"
#include "middleware/moves/basicMoves/basicMoves.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/settings/settings.h"

//Declarations for this module */
#include "middleware/moves/mazeMoves/spyPost.h"

// Index of left profile in the array
#define SPYPOST_LEFT_PROFILE_IDX    0
// Index of right profile in the array
#define SPYPOST_RIGHT_PROFILE_IDX   1

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM     4
#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	   64
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	   40
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	   ((SPYPOST_TELEMETER_STEPS_MEASURE_MM * SPYPOST_NBITS_SAMPLING_RESOLUTION) + SPYPOST_MIN_DIAG_SENSOR_DISTANCE)

#define SPYPOST_REFERENCE_SAMPLE_HEIGHT 	   16	//16 bit height
#define SPYPOST_REFERENCE_SAMPLE_WIDTH 		   40	//array length

#define SPYPOST_MOVE_SPEED 					   100

#define MIN_STAT                               65   //minimum percentage for validate
#define DIST_FOR_TIME_CALCULATION              20   //stop record before end to have time to calculate

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

spyPostRefProfileStruct *ref_left = NULL;
spyPostRefProfileStruct *ref_right = NULL;

/* Static functions */
static void spyPostStartMeasure(spyPostProfileStruct *currentProfile, enum telemeterName telemeterName);
static void spyPostPrintProfile(uint32_t x, uint32_t y, uint64_t *sample, uint32_t center_x_distance, uint32_t center_y_distance);
static void spyPostSendBTProfile(uint32_t *buf32, uint32_t lenght_buf32);
static void spyPostKeepUsefulPart(spyPostProfileStruct *typeProfile);
static void spyPostSampleThicken(spyPostProfileStruct *profile);


int spyPostInit(void)
{
    int rv;

    ref_left  = &(zhonxCalib_data->spyPost[0]);
    ref_right = &(zhonxCalib_data->spyPost[1]);

    // Check whether Left and Right profiles are initialized
    if ((ref_left->initializer != 0xDEADBEEF) || (ref_right->initializer != 0xDEADBEEF))
    {
        return SPYPOST_DRIVER_E_NOT_CALIBRATED;
    }

    return SPYPOST_DRIVER_E_SUCCESS;
}


/*
 *
 *      o         o
 *      :    |    :
 *      :    |    :
 *      o         o
 */
uint32_t spyPostGetOffset(spyPostGetOffsetsStruct *offset)
{
    uint32_t i;
    uint32_t left_sample = 0;
    uint32_t right_sample = 0;
    spyPostProfileStruct current_left;
    spyPostProfileStruct current_right;

    //clear stats
    uint32_t left_wallToNoWall_stat = 0;
    uint32_t left_singlePost_stat = 0;
    uint32_t left_perpendicularWall_stat = 0;
    uint32_t right_wallToNoWall_stat = 0;
    uint32_t right_singlePost_stat = 0;
    uint32_t right_perpendicularWall_stat = 0;

    memset(&current_right.sample, 0, sizeof(current_right.sample));
    memset(&current_left.sample, 0, sizeof(current_left.sample));

    offset->left_x = 0;
    offset->right_x = 0;

    char left_wall_presence = FALSE;
    char right_wall_presence = FALSE;

    if (mainControlGetFollowType() != WALL_FOLLOW)
    {
        return SPYPOST_DRIVER_E_SUCCESS;
    }

    if (getWallPresence(LEFT_WALL) == TRUE)
    {
        left_wall_presence = TRUE;
    }
    if (getWallPresence(RIGHT_WALL) == TRUE)
    {
        right_wall_presence = TRUE;
    }

    //take the measures
    for (i = 0;
            i < (SPYPOST_ARRAY_PROFILE_LENGTH -
                    (uint32_t)((OFFSET_DIST * 2) + DIST_FOR_TIME_CALCULATION) / SPYPOST_ENCODERS_STEPS_MEASURE_MM);
            i++)
    {
        while (((uint32_t)(encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i)));

        left_sample = (uint32_t)getTelemeterDist(TELEMETER_DL);
        right_sample = (uint32_t)getTelemeterDist(TELEMETER_DR);
        // left statement
        if (((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < SPYPOST_NBITS_SAMPLING_RESOLUTION)
        {
            current_left.sample[i] = 1 << ((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        }
        else
        {
            current_left.sample[i] = 0x00;
        }

        // right statement
        if (((right_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < SPYPOST_NBITS_SAMPLING_RESOLUTION)
        {
            current_right.sample[i] = 1 << ((right_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        }
        else
        {
            current_right.sample[i] = 0x00;
        }
    }
    spyPostKeepUsefulPart(&current_left);
    spyPostKeepUsefulPart(&current_right);

    //Compute statistic and apply the correct offset
    //left part
    if (left_wall_presence == TRUE)
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
            if (current_left.sample[i] & ref_left->wallToNoWall.sample[i])
            {
                left_wallToNoWall_stat++;
            }
        }
        left_wallToNoWall_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
        if (left_wallToNoWall_stat > MIN_STAT)
        {
            offset->left_x = current_left.center_x_distance - ref_left->wallToNoWall.center_x_distance;
            offset->left_y = current_left.center_y_distance - ref_left->wallToNoWall.center_y_distance;
            offset->left_spyPostType = WALL_TO_NO_WALL;
        }
#ifdef DEBUG_SPYPOST
        bluetoothWaitReady();
        bluetoothPrintf("\nL_WTNW x = %d, y = %d, stat = %d", (int32_t)offset->left_x,
                        (uint32_t)offset->left_y, (uint32_t)left_wallToNoWall_stat);
#endif
    }
    else
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
#ifdef ENABLE_SINGLE_POST
            if (current_left.sample[i] & ref_left->singlePost.sample[i])
            {
                left_singlePost_stat++;
            }
#endif
            if (current_left.sample[i] & ref_left->perpendicularWall.sample[i])
            {
                left_perpendicularWall_stat++;
            }
        }
        left_perpendicularWall_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
#ifdef ENABLE_SINGLE_POST
        left_singlePost_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
        if (left_singlePost_stat > left_perpendicularWall_stat)
        {
            if (left_singlePost_stat > MIN_STAT)
            {
                offset->left_x = current_left.center_x_distance - ref_left->singlePost.center_x_distance;
                offset->left_y = current_left.center_y_distance - ref_left->singlePost.center_y_distance;
                offset->left_spyPostType = SINGLE_POST;
            }
#ifdef DEBUG_SPYPOST
            bluetoothWaitReady();
            bluetoothPrintf("\nL_SP x = %d, y = %d, stat = %d", (int32_t)offset->left_x,
                            (uint32_t)offset->left_y, (uint32_t)left_singlePost_stat);
#endif
        }
        else
#endif
        {
            if (left_perpendicularWall_stat >= MIN_STAT)
            {
                offset->left_x = current_left.center_x_distance - ref_left->perpendicularWall.center_x_distance;
                offset->left_y = current_left.center_y_distance - ref_left->perpendicularWall.center_y_distance;
                offset->left_spyPostType = PERPENDICULAR_WALL;
            }
#ifdef DEBUG_SPYPOST
#ifdef ENABLE_SINGLE_POST
            if (left_singlePost_stat == left_perpendicularWall_stat)
            {
                bluetoothWaitReady();
                bluetoothPrintf("\nSAME L PW SP, ");
            }
#endif
            bluetoothWaitReady();
            bluetoothPrintf("\nL_PW x = %d, y = %d, stat = %d", (int32_t)offset->left_x,
                            (uint32_t)offset->left_y, (uint32_t)left_perpendicularWall_stat);
#endif
        }
    }
    //right part
    if (right_wall_presence == TRUE)
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
            if (current_right.sample[i] & ref_right->wallToNoWall.sample[i])
            {
                right_wallToNoWall_stat++;
            }
        }
        right_wallToNoWall_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
        if (right_wallToNoWall_stat > MIN_STAT)
        {
            offset->right_x = current_right.center_x_distance - ref_right->wallToNoWall.center_x_distance;
            offset->right_y = current_right.center_y_distance - ref_right->wallToNoWall.center_y_distance;
            offset->right_spyPostType = WALL_TO_NO_WALL;
        }
#ifdef DEBUG_SPYPOST
        bluetoothWaitReady();
        bluetoothPrintf("\nR_WTNW x = %d, y = %d, stat = %d", (int32_t)offset->right_x,
                        (uint32_t)offset->right_y, (uint32_t)right_wallToNoWall_stat);
#endif
    }
    else
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
#ifdef ENABLE_SINGLE_POST
            if (current_right.sample[i] & ref_right->singlePost.sample[i])
            {
                right_singlePost_stat++;
            }
#endif
            if (current_right.sample[i] & ref_right->perpendicularWall.sample[i])
            {
                right_perpendicularWall_stat++;
            }
        }
        right_perpendicularWall_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
#ifdef ENABLE_SINGLE_POST
        right_singlePost_stat *= 100 / SPYPOST_REFERENCE_SAMPLE_WIDTH;
        if (right_singlePost_stat > right_perpendicularWall_stat)
        {
            if (right_singlePost_stat > MIN_STAT)
            {
                offset->right_x = current_right.center_x_distance - ref_right->singlePost.center_x_distance;
                offset->right_y = current_right.center_y_distance - ref_right->singlePost.center_y_distance;
                offset->right_spyPostType = SINGLE_POST;
            }
#ifdef DEBUG_SPYPOST
            bluetoothWaitReady();
            bluetoothPrintf("\nR_SP x = %d, y = %d, stat = %d", (int32_t)offset->right_x,
                            (uint32_t)offset->right_y, (uint32_t)right_singlePost_stat);
#endif
        }
        else
#endif
        {
            if (right_perpendicularWall_stat >= MIN_STAT)
            {
                offset->right_x = current_right.center_x_distance - ref_right->perpendicularWall.center_x_distance;
                offset->right_y = current_right.center_y_distance - ref_right->perpendicularWall.center_y_distance;
                offset->right_spyPostType = PERPENDICULAR_WALL;
            }
#ifdef DEBUG_SPYPOST
#ifdef ENABLE_SINGLE_POST
            if (right_singlePost_stat == right_perpendicularWall_stat)
            {
                bluetoothWaitReady();
                bluetoothPrintf("\nSAME R PW SP, ");
            }
#endif
            bluetoothWaitReady();
            bluetoothPrintf("\nR_PW x = %d, y = %d, stat = %d", (int32_t)offset->right_x,
                            (uint32_t)offset->right_y, (uint32_t)right_perpendicularWall_stat);
#endif
        }
    }
#ifdef DEBUG_SPYPOST
    spyPostPrintProfile(0, 64, current_left.sample,current_left.center_x_distance, current_left.center_y_distance);
    spyPostPrintProfile(43, 64, current_right.sample, current_right.center_x_distance, current_right.center_y_distance);
#endif
    return SPYPOST_DRIVER_E_SUCCESS;
}

uint32_t spyPostCalibration(void)
{
    spyPostRefProfileStruct refProfiles_ram;
    spyPostProfileStruct    currentProfile;
    uint32_t                i;
    spyPostRefProfileStruct *refProfile_flash = NULL;
    int                     rv = 0;

    // Initialize structure
    refProfiles_ram.initializer = 0;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while ((expanderJoyState() != JOY_RIGHT) || (refProfile_flash == NULL))
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
#ifdef ENABLE_SINGLE_POST
            ssd1306DrawBmp(spyPostRight_Img, 1, 24, 128, 40);
#else
            ssd1306DrawBmp(spyPostRightLight_Img, 6, 24, 115, 40);
#endif
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfile_flash = ref_right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
#ifdef ENABLE_SINGLE_POST
            ssd1306DrawBmp(spyPostLeft_Img, 1, 24, 128, 40);
#else
            ssd1306DrawBmp(spyPostLeftLight_Img, 6, 24, 115, 40);
#endif
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfile_flash = ref_left;
        }
        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return SPYPOST_DRIVER_E_SUCCESS;
        }
    }

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    mainControlSetFollowType(NO_FOLLOW);
    positionControlSetPositionType(ENCODERS);
    HAL_Delay(4000);
    telemetersStart();

#ifdef ENABLE_SINGLE_POST
    for (i = 0; i < 3; i++)
#else
        for (i = 0; i < 2; i++)
#endif
        {
            if (refProfile_flash == ref_left)
                spyPostStartMeasure(&currentProfile, TELEMETER_DL);
            else if (refProfile_flash == ref_right)
                spyPostStartMeasure(&currentProfile, TELEMETER_DR);
            spyPostKeepUsefulPart(&currentProfile);
            if (currentProfile.center_x_distance != 0)
                currentProfile.center_x_distance -= OFFSET_DIST; //removed OFFSET DIST for calib, see spyPostKeepUsefulPart
            spyPostSampleThicken(&currentProfile);

            switch (i)
            {
                case 0:
                    memcpy(&refProfiles_ram.wallToNoWall, &currentProfile, sizeof(spyPostProfileStruct));
                    break;
#ifdef ENABLE_SINGLE_POST
                case 1:
                    if (refProfile_flash == ref_left)
                        memcpy(&refProfiles_ram.singlePost, &currentProfile, sizeof(spyPostProfileStruct));
                    else
                        memcpy(&refProfiles_ram.perpendicularWall, &currentProfile, sizeof(spyPostProfileStruct));
                    break;

                case 2:
                    if (refProfile_flash == ref_left)
                        memcpy(&refProfiles_ram.perpendicularWall, &currentProfile, sizeof(spyPostProfileStruct));
                    else
                        memcpy(&refProfiles_ram.singlePost, &currentProfile, sizeof(spyPostProfileStruct));
                    break;
#else
                case 1:
                    memcpy(&refProfiles_ram.perpendicularWall, &currentProfile, sizeof(spyPostProfileStruct));
                    break;
#endif
            }
        }
    telemetersStop();
    basicMoveStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
    // Save calibration parameters to flash memory
    refProfiles_ram.initializer = 0xDEADBEEF;
    rv = flash_write(zhonxSettings.h_flash, (unsigned char *)refProfile_flash,
                     (unsigned char *)&refProfiles_ram, sizeof(spyPostRefProfileStruct));
    if (rv != FLASH_E_SUCCESS)
    {
        bluetoothPrintf("Error writing into flash memory (%d)", rv);
        return rv;
    }

    spyPostPrintProfile(0, 64, refProfile_flash->wallToNoWall.sample, refProfile_flash->wallToNoWall.center_x_distance, refProfile_flash->wallToNoWall.center_y_distance);
#ifdef ENABLE_SINGLE_POST
    spyPostPrintProfile(43, 64, refProfile_flash->singlePost.sample, refProfile_flash->singlePost.center_x_distance, refProfile_flash->singlePost.center_y_distance);
#endif
    spyPostPrintProfile(86, 64, refProfile_flash->perpendicularWall.sample, refProfile_flash->perpendicularWall.center_x_distance, refProfile_flash->perpendicularWall.center_y_distance);

    //send BT profiles
    bluetoothPrintf("\n wallToNoWall x = %d\n", refProfile_flash->wallToNoWall.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->wallToNoWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
#ifdef ENABLE_SINGLE_POST
    bluetoothPrintf("\n singlePost x = %d\n", refProfile_flash->singlePost.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->singlePost.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
#endif
    bluetoothPrintf("\n perpendicularWall x = %d\n", refProfile_flash->perpendicularWall.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->perpendicularWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);

    while (expanderJoyFiltered() != JOY_LEFT);
    return SPYPOST_DRIVER_E_SUCCESS;
}

/*
 *      o    |    o
 *      :    |    :
 *      :    |    :
 *      o    |    o
 */
void spyPostStartMeasure(spyPostProfileStruct *currentProfile, enum telemeterName telemeterName)
{
    //sanity check
    if (telemeterName != TELEMETER_DR && telemeterName != TELEMETER_DL)
        return;

    uint32_t i = 0;
    uint32_t sample = 0;

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    wallFollowSetInitialPosition(0.00); //absolute position into a cell

    //take the measures
    basicMove(0, SPYPOST_CAL_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((uint32_t)(encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) && (hasMoveEnded() != TRUE));

        sample = (uint32_t)getTelemeterDist(telemeterName);

        if (((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < SPYPOST_NBITS_SAMPLING_RESOLUTION)
        {
            currentProfile->sample[i] = (uint64_t)0x01 << (uint64_t)(((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM));
        }
        else
            currentProfile->sample[i] = 0x00;

        if (i % 20 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintIntAtLine(0, 0, "wall dist :  ", (uint32_t)sample, &Font_5x8);
            ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", (uint32_t)(currentProfile->sample[i] / 2));
            ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (uint32_t)(SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1), &Font_5x8);
            ssd1306ProgressBar(10, 50, (i * 100) / SPYPOST_ARRAY_PROFILE_LENGTH);
            ssd1306Refresh();
        }
    }
    while (hasMoveEnded() != TRUE);
    ssd1306ClearScreen(MAIN_AREA);
}

void spyPostPrintProfile(uint32_t x, uint32_t y, uint64_t *sample, uint32_t center_x_distance, uint32_t center_y_distance)
{
    double x_coeff = (40.00 / SPYPOST_REFERENCE_SAMPLE_WIDTH);
    ssd1306DrawRect(0 + x, y - 4 - (SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2),
                    (uint32_t)(SPYPOST_REFERENCE_SAMPLE_WIDTH * x_coeff) + 2,
                    SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2 + 4);
    ssd1306PrintfAtLine(x + 5, 1, &Font_3x6, "%d\n", (uint32_t)center_x_distance);
    ssd1306PrintfAtLine(x + 30, 1, &Font_3x6, "%d\n", (uint32_t)center_y_distance);
    for (uint64_t i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        for (uint64_t j = SPYPOST_REFERENCE_SAMPLE_HEIGHT; j > 0; j--)
        {
            if ((sample[i] >> (j)) & 0x01)
            {
                ssd1306DrawPixel((uint32_t)(i * x_coeff) + x + 1, y - j * 2 - 1);
            }
        }
    }
    ssd1306Refresh();
}

void spyPostSendBTProfile(uint32_t *buf32, uint32_t lenght_buf32)
{
    uint32_t i = 0;
    for (i = 0; i < lenght_buf32; i++)
    {
        bluetoothWaitReady();
        bluetoothPrintf("%d, ", (buf32[i]));
    }
    bluetoothWaitReady();
}

/*
 *      | OD             MAIN DIST SCAN               OD|
 *      |-->|---------------------------------------|-->|
 *      |                                     _.--      |
 *      |    .-----._                       .'          |
 *      |            '-._         ____x___ /            |
 *      |                '-._     |   |   /             |
 *      |                    '-._ |___|__/|y            |
 *      |                        '-._ |.' |             |
 *      |                         |___|___|             |
 *      |                                               |
 *
 *      |<--------------SCAN REFERENCE----------------->|
 *      |OD>|<---------------SCAN COMPARE-----------~~~~~~~>|
 *                                                      |OD>|
 *                                     <-------x--------|
 */
void spyPostKeepUsefulPart(spyPostProfileStruct *currentProfile)
{
    uint64_t i, j;
    uint64_t y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
    uint64_t mask = (uint64_t)(pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);

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
    //sanity check
    if ((SPYPOST_NBITS_SAMPLING_RESOLUTION - y_min) < SPYPOST_REFERENCE_SAMPLE_HEIGHT)
    {
        currentProfile->center_x_distance = 0; //Clear deprecated values
        currentProfile->center_y_distance = 0;
        //        ssd1306PrintIntAtLine(0, 0, &Font_5x8, " SPYPOST ERROR SAMPLING HEIGHT - y_min) < REFERENCE_SAMPLE_HEIGHT");
        return;
    }
    //save the central y distance
    currentProfile->center_y_distance = (y_min + (SPYPOST_REFERENCE_SAMPLE_HEIGHT / 2)) * SPYPOST_TELEMETER_STEPS_MEASURE_MM;
    //shift buffer (y axis)
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] >>= (y_min - 1); //one pix margin
    }
    //crop buffer height
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] &= mask;
    }
    //search the last value uint32_to the buffer for align reference sample
    i = SPYPOST_ARRAY_PROFILE_LENGTH - 1;
    while (currentProfile->sample[i] == 0)
    {
        //sanity check
        if (i < SPYPOST_REFERENCE_SAMPLE_WIDTH)
        {
            currentProfile->center_x_distance = 0; //Clear deprecated values
            currentProfile->center_y_distance = 0;
            //        ssd1306PrintIntAtLine(0, 0, &Font_5x8, " SPYPOST ERROR x < 0");
            return;
        }
        i--;
    }
    //save the central x distance
    currentProfile->center_x_distance = ((i - (SPYPOST_REFERENCE_SAMPLE_WIDTH / 2)) * SPYPOST_ENCODERS_STEPS_MEASURE_MM) + OFFSET_DIST;
    //move useful part of sample at the beginning of the buffer
    memmove(currentProfile->sample, currentProfile->sample + (i - SPYPOST_REFERENCE_SAMPLE_WIDTH),
            SPYPOST_REFERENCE_SAMPLE_WIDTH * 8);
    //Clear the rest of the buffer after move
    memset(currentProfile->sample + SPYPOST_REFERENCE_SAMPLE_WIDTH, 0,
           (SPYPOST_ARRAY_PROFILE_LENGTH - SPYPOST_REFERENCE_SAMPLE_WIDTH) * 8);
}

void spyPostSampleThicken(spyPostProfileStruct *profile)
{
    uint32_t i, j;
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        profile->sample[i] |= (uint16_t)(profile->sample[i] << 1);
        profile->sample[i] |= (uint16_t)(profile->sample[i] >> 1);
    }
}

uint32_t spyPostReadCalibration(void)
{
    spyPostRefProfileStruct *refProfiles = NULL;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (1)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = ref_right;
            spyPostPrintProfile(0, 64, refProfiles->wallToNoWall.sample, refProfiles->wallToNoWall.center_x_distance, refProfiles->wallToNoWall.center_y_distance);
#ifdef ENABLE_SINGLE_POST
            spyPostPrintProfile(43, 64, refProfiles->singlePost.sample, refProfiles->singlePost.center_x_distance, refProfiles->singlePost.center_y_distance);
#endif
            spyPostPrintProfile(86, 64, refProfiles->perpendicularWall.sample,
                                refProfiles->perpendicularWall.center_x_distance, refProfiles->perpendicularWall.center_y_distance);
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = ref_left;
            spyPostPrintProfile(0, 64, refProfiles->wallToNoWall.sample, refProfiles->wallToNoWall.center_x_distance, refProfiles->wallToNoWall.center_y_distance);
#ifdef ENABLE_SINGLE_POST
            spyPostPrintProfile(43, 64, refProfiles->singlePost.sample, refProfiles->singlePost.center_x_distance, refProfiles->singlePost.center_y_distance);
#endif
            spyPostPrintProfile(86, 64, refProfiles->perpendicularWall.sample,
                                refProfiles->perpendicularWall.center_x_distance, refProfiles->perpendicularWall.center_y_distance);
        }
        if (expanderJoyFiltered() == JOY_LEFT)
            return SPYPOST_DRIVER_E_SUCCESS;
    }

    return SPYPOST_DRIVER_E_SUCCESS;
}

void spyPostTest()
{
    uint32_t Vmin, Vmax, Vrotate;

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawBmp(spyPostTest_Img, 25, 24, 74, 31);
        ssd1306DrawStringAtLine(35, 0, "SPYPOST TEST", &Font_3x6);
        ssd1306Refresh();

        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(100);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    HAL_Delay(2000);
    ssd1306ClearScreen(MAIN_AREA);
    telemetersStart();

    Vmin = 200;
    Vmax = 200;

    basicMove(0, OFFSET_DIST, Vmax, Vmax);
    while (hasMoveEnded() != TRUE);
    mazeMoveCell(1, Vmax, Vmin);
    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
}
