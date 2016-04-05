/*
 * spyPost.c
 *
 *  Created on: 19 mars 2016
 *      Author: zhonx
 *  V1.1
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
#include "middleware/controls/mazeControl/reposition.h"

//Declarations for this module */
#include "middleware/controls/mazeControl/spyPost.h"

// Index of left profile in the array
#define SPYPOST_LEFT_PROFILE_IDX    0
// Index of right profile in the array
#define SPYPOST_RIGHT_PROFILE_IDX   1

#define SPYPOST_ENCODERS_STEPS_MEASURE_MM 	   1
#define SPYPOST_CAL_DISTANCE			       (uint32_t)MAIN_DIST
#define SPYPOST_ARRAY_PROFILE_LENGTH 		   ((SPYPOST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

#define SPYPOST_TELEMETER_STEPS_MEASURE_MM     2
#define SPYPOST_NBITS_SAMPLING_RESOLUTION 	   32
#define SPYPOST_MIN_DIAG_SENSOR_DISTANCE 	   65
#define SPYPOST_MAX_DIAG_SENSOR_DISTANCE 	   (SPYPOST_TELEMETER_STEPS_MEASURE_MM * SPYPOST_NBITS_SAMPLING_RESOLUTION) + SPYPOST_MIN_DIAG_SENSOR_DISTANCE

#define SPYPOST_REFERENCE_SAMPLE_HEIGHT 	   16	//16 bit height
#define SPYPOST_REFERENCE_SAMPLE_WIDTH 		   20	//array length

#define SPYPOST_MOVE_SPEED 					   50

#define MIN_STAT                               16

#if (SPYPOST_MAX_DIAG_SENSOR_DISTANCE - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) % (SPYPOST_NBITS_SAMPLING_RESOLUTION) != 0
#error  MAX DIAG - MIN_DIAG must be a multiple of SAMPLING_RESOLUTION
#endif

// Machine Definitions
typedef struct
{
    uint32_t sample[SPYPOST_ARRAY_PROFILE_LENGTH + 1];
    uint32_t center_x_distance;
    uint32_t center_y_distance;
} spyPostProfileStruct;

typedef struct
{
    spyPostProfileStruct wallToNoWall;
    spyPostProfileStruct singlePost;
    spyPostProfileStruct perpendicularWall;
    unsigned int         initializer;
} spyPostRefTypeProfileStruct;

//spyPostRefTypeProfileStruct ref_left = //todo add saved value uint32_to flash
//{ .wallToNoWall =
//{ .sample =
//{ 14, 28, 14, 28, 28, 28, 28, 28, 56, 56, 56, 56, 112, 224, 224, 448, 896, 1792, 3584, 14336 }, .center_x_distance =
//        58 }, .singlePost =
//        { .sample = { 896, 224, 112, 56, 28, 14, 7, 7, 7, 7, 7, 7, 14, 56, 56, 112, 224, 896, 3584, 28672 }, .center_x_distance =
//                62 }, .perpendicularWall =
//                { .sample = { 224, 224, 56, 56, 28, 28, 14, 14, 14, 7, 14, 14, 14, 28, 56, 112, 224, 896, 3584, 28672 }, .center_x_distance =
//                        62 } };
//
//spyPostRefTypeProfileStruct ref_right = //todo add saved value uint32_to flash
//{ .wallToNoWall =
//{ .sample =
//{ 14, 14, 14, 14, 14, 14, 14, 14, 28, 28, 28, 56, 56, 112, 224, 448, 896, 1792, 7168, 28672 }, .center_x_distance =
//        56 }, .singlePost =
//        { .sample =
//        { 7168, 1792, 896, 448, 112, 56, 28, 14, 14, 14, 7, 7, 14, 14, 28, 112, 224, 896, 3584, 14336 }, .center_x_distance =
//                59 }, .perpendicularWall =
//                { .sample = { 448, 112, 56, 28, 28, 14, 7, 7, 7, 7, 7, 7, 14, 28, 56, 224, 448, 1792, 7168, 28672 }, .center_x_distance =
//                        60 }, };

// Declare Reference SpyPost profiles in Flash memory
spyPostRefTypeProfileStruct *spyPost_profiles = (spyPostRefTypeProfileStruct *)ADDR_FLASH_SECTOR_9;

spyPostRefTypeProfileStruct *ref_left = NULL;
spyPostRefTypeProfileStruct *ref_right = NULL;

/* Static functions */
static void spyPostStartMeasure(spyPostProfileStruct *currentProfile, enum telemeterName telemeterName);
static void spyPostPrintProfile(uint32_t x, uint32_t y, uint32_t *sample, uint32_t center_x_distance);
static void spyPostSendBTProfile(uint32_t *buf32, uint32_t lenght_buf32);
static void spyPostKeepUsefulPart(spyPostProfileStruct *typeProfile);
static void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width);


int spyPostInit(void)
{
    int rv;

    ref_left  = spyPost_profiles;
    ref_right = ((unsigned char *)spyPost_profiles) + sizeof(spyPostRefTypeProfileStruct);

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
    uint8_t left_wallToNoWall_stat = 0;
    uint8_t left_singlePost_stat = 0;
    uint8_t left_perpendicularWall_stat = 0;
    uint8_t right_wallToNoWall_stat = 0;
    uint8_t right_singlePost_stat = 0;
    uint8_t right_perpendicularWall_stat = 0;

    memset(&current_right.sample, 0, sizeof(current_right.sample));
    memset(&current_left.sample, 0, sizeof(current_left.sample));

    offset->left_x = 0;
    offset->right_x = 0;

    char left_wall_presence = FALSE;
    char right_wall_presence = FALSE;

    if (getWallPresence(LEFT_WALL) == TRUE)
    {
        left_wall_presence = TRUE;
    }
    if (getWallPresence(RIGHT_WALL) == TRUE)
    {
        right_wall_presence = TRUE;
    }

    //take the measures
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((uint32_t) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) && (hasMoveEnded() != TRUE))
        {
        }
        left_sample = (uint32_t) getTelemeterDist(TELEMETER_DL);
        right_sample = (uint32_t) getTelemeterDist(TELEMETER_DR);
        // left statement
        if (((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < 32)
        {
            current_left.sample[i] = 1 << ((left_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        }
        else
        {
            current_left.sample[i] = 0x00;
        }

        // right statement
        if (((right_sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < 32)
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
        if (left_wallToNoWall_stat > MIN_STAT)
        {
            offset->left_x = ref_left->wallToNoWall.center_x_distance - current_left.center_x_distance;
            offset->left_y = ref_left->wallToNoWall.center_y_distance - current_left.center_y_distance;
        }
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("L_WTNW x = %d, L_WTNW y = %d, stat = %d\n", (int32_t) offset->left_x,
                        (uint32_t) offset->left_y, (int32_t) left_wallToNoWall_stat);
#endif
    }
    else
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
            if (current_left.sample[i] & ref_left->singlePost.sample[i])
            {
                left_singlePost_stat++;
            }
            if (current_left.sample[i] & ref_left->perpendicularWall.sample[i])
            {
                left_perpendicularWall_stat++;
            }
        }
        if (left_singlePost_stat > left_perpendicularWall_stat)
        {
            if (left_singlePost_stat > MIN_STAT)
            {
                offset->left_x = ref_left->singlePost.center_x_distance - current_left.center_x_distance;
                offset->left_y = ref_left->singlePost.center_y_distance - current_left.center_y_distance;
            }
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("L_SP x = %d, L_SP y = %d, stat = %d\n", (int32_t) offset->left_x,
                            (uint32_t) offset->left_y, (int32_t) left_singlePost_stat);
#endif
        }
        else
        {
            if (left_perpendicularWall_stat >= MIN_STAT)
            {
                offset->left_x = ref_left->perpendicularWall.center_x_distance - current_left.center_x_distance;
                offset->left_y = ref_left->perpendicularWall.center_y_distance - current_left.center_y_distance;
            }
#ifdef DEBUG_SPYPOST
            if (left_singlePost_stat == left_perpendicularWall_stat)
            {
                bluetoothPrintf("SAME L PW SP, ");
                bluetoothWaitReady();
            }
            bluetoothPrintf("L_PW x = %d, L_PW y = %d, stat = %d\n", (int32_t) offset->left_x,
                            (uint32_t) offset->left_y, (int32_t) left_perpendicularWall_stat);
#endif
        }
    }
    //Delay for BT debug
#ifdef DEBUG_SPYPOST
    bluetoothWaitReady();
#endif
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
        if (right_wallToNoWall_stat > MIN_STAT)
        {
            offset->right_x = ref_right->wallToNoWall.center_x_distance - current_right.center_x_distance;
            offset->right_y = ref_right->wallToNoWall.center_y_distance - current_right.center_y_distance;
        }
#ifdef DEBUG_SPYPOST
        bluetoothPrintf("R_WTNW x = %d, R_WTNW = y %d, stat = %d\n", (int32_t) offset->right_x,
                        (uint32_t) offset->right_y, (int32_t) right_wallToNoWall_stat);
#endif
    }
    else
    {
        for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
        {
            if (current_right.sample[i] & ref_right->singlePost.sample[i])
            {
                right_singlePost_stat++;
            }
            if (current_right.sample[i] & ref_right->perpendicularWall.sample[i])
            {
                right_perpendicularWall_stat++;
            }
        }
        if (right_singlePost_stat > right_perpendicularWall_stat)
        {
            if (right_singlePost_stat > MIN_STAT)
            {
                offset->right_x = ref_right->singlePost.center_x_distance - current_right.center_x_distance;
                offset->right_y = ref_right->singlePost.center_y_distance - current_right.center_y_distance;
            }
#ifdef DEBUG_SPYPOST
            bluetoothPrintf("R_SP x = %d, R_SP y = %d, stat = %d\n", (int32_t) offset->right_x,
                            (uint32_t) offset->right_y, (int32_t) right_singlePost_stat);
#endif
        }
        else
        {
            if (right_perpendicularWall_stat >= MIN_STAT)
            {
                offset->right_x = ref_right->perpendicularWall.center_x_distance - current_right.center_x_distance;
                offset->right_y = ref_right->perpendicularWall.center_y_distance - current_right.center_y_distance;
            }
#ifdef DEBUG_SPYPOST
            if (right_singlePost_stat == right_perpendicularWall_stat)
            {
                bluetoothPrintf("SAME R PW SP, ");
                bluetoothWaitReady();
            }
            bluetoothPrintf("R_PW x = %d, R_PW y = %d, stat = %d\n", (int32_t) offset->right_x,
                            (uint32_t) offset->right_y, (int32_t) right_perpendicularWall_stat);
#endif
        }
    }
    //    spyPostPrintProfile(0, 64, current_left.sample, current_left.center_x_distance);
    //    spyPostPrintProfile(43, 64, current_right.sample, current_right.center_x_distance);
    return SPYPOST_DRIVER_E_SUCCESS;
}

uint32_t spyPostCalibration(void)
{
    spyPostRefTypeProfileStruct refProfiles_ram;
    spyPostProfileStruct        currentProfile;
    uint32_t                    i;
    spyPostRefTypeProfileStruct *refProfile_flash = NULL;
    int                         rv = 0;

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
            ssd1306DrawBmp(spyPostRight, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
//            refProfiles = ref_right;
            refProfile_flash = ref_right;
            //memset((spyPostRefTypeProfileStruct*) ref_right, 0, sizeof(spyPostRefTypeProfileStruct));   //reinit stucts
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(spyPostLeft, 1, 24, 128, 40);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
//            refProfiles = ref_left;
            refProfile_flash = ref_left;
            //memset((spyPostRefTypeProfileStruct*) ref_left, 0, sizeof(spyPostRefTypeProfileStruct));    //reinit stucts
        }
        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return SPYPOST_DRIVER_E_SUCCESS;
        }
    }

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(40, 1, "Wait", &Font_3x6);
    ssd1306Refresh();

    mainControlInit();
    mainControlSetFollowType(NO_FOLLOW);
    HAL_Delay(4000);

    for (i = 0; i < 3; i++)
    {
        if (refProfile_flash == ref_left)
        {
            spyPostStartMeasure(&currentProfile, TELEMETER_DL);
        }
        else
        {
            spyPostStartMeasure(&currentProfile, TELEMETER_DR);
        }

        spyPostKeepUsefulPart(&currentProfile);
        spyPostSampleThicken(&currentProfile, 2);

        switch (i)
        {
            case 0:
                memcpy(&refProfiles_ram.wallToNoWall, &currentProfile, sizeof(spyPostProfileStruct));
                break;

            case 1:
                if (refProfile_flash == ref_left)
                {
                    memcpy(&refProfiles_ram.singlePost, &currentProfile, sizeof(spyPostProfileStruct));
                }
                else
                {
                    memcpy(&refProfiles_ram.perpendicularWall, &currentProfile, sizeof(spyPostProfileStruct));
                }
                break;

            case 2:
                if (refProfile_flash == ref_left)
                {
                    memcpy(&refProfiles_ram.perpendicularWall, &currentProfile, sizeof(spyPostProfileStruct));
                }
                else
                {
                    memcpy(&refProfiles_ram.singlePost, &currentProfile, sizeof(spyPostProfileStruct));
                }
                break;
        }
    }

    // Save calibration parameters to flash memory
    refProfiles_ram.initializer = 0xDEADBEEF;
    rv = flash_write(zhonxSettings.h_flash, (unsigned char *)refProfile_flash,
                     (unsigned char *)&refProfiles_ram, sizeof(spyPostRefTypeProfileStruct));
    if (rv != FLASH_E_SUCCESS)
    {
        bluetoothPrintf("Error writing into flash memory (%d)", rv);
        return rv;
    }

    spyPostPrintProfile(0, 64, refProfile_flash->wallToNoWall.sample, refProfile_flash->wallToNoWall.center_x_distance);
    spyPostPrintProfile(43, 64, refProfile_flash->singlePost.sample, refProfile_flash->singlePost.center_x_distance);
    spyPostPrintProfile(86, 64, refProfile_flash->perpendicularWall.sample,
                        refProfile_flash->perpendicularWall.center_x_distance);

    //send BT profiles
    bluetoothPrintf("\n wallToNoWall x = %d\n", refProfile_flash->wallToNoWall.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->wallToNoWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("\n singlePost x = %d\n", refProfile_flash->singlePost.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->singlePost.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);
    bluetoothPrintf("\n perpendicularWall x = %d\n", refProfile_flash->perpendicularWall.center_x_distance);
    spyPostSendBTProfile((uint32_t*) &refProfile_flash->perpendicularWall.sample, SPYPOST_REFERENCE_SAMPLE_WIDTH);

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

    uint32_t i = 0;
    uint32_t sample = 0;
    telemetersStart();

    //offset dist
    repositionSetInitialPosition(0); //absolute position into a cell
    move(0, OFFSET_DIST, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    while (hasMoveEnded() != TRUE)
    {
    }
//    mainControlSetFollowType(WALL_FOLLOW);
    repositionSetInitialPosition(OFFSET_DIST); //absolute position into a cell
    //take the measures
    move(0, SPYPOST_CAL_DISTANCE, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        while ((((uint32_t) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R))
                <= (SPYPOST_ENCODERS_STEPS_MEASURE_MM * 2 * i))) && (hasMoveEnded() != TRUE))
            ;

        sample = (uint32_t) getTelemeterDist(telemeterName);

        if (((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) / SPYPOST_TELEMETER_STEPS_MEASURE_MM) < 32)
        {
            currentProfile->sample[i] = 0x01 << ((sample - SPYPOST_MIN_DIAG_SENSOR_DISTANCE) /
                    SPYPOST_TELEMETER_STEPS_MEASURE_MM);
        }
        else
            currentProfile->sample[i] = 0x00;

        if (i % 2 == 1)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306PrintfAtLine(0, 1, &Font_5x8, "bin : %d", (uint32_t*) currentProfile->sample[i]);
            ssd1306PrintIntAtLine(0, 0, "wall dist :  ", (uint32_t) sample, &Font_5x8);
            ssd1306PrintIntAtLine(0, 2, "enc. dist :  ", (uint32_t) (SPYPOST_ENCODERS_STEPS_MEASURE_MM * i + 1),
                                  &Font_5x8);
            ssd1306ProgressBar(10, 50, (i * 100) / SPYPOST_ARRAY_PROFILE_LENGTH);
            ssd1306Refresh();
        }
    }
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition(SPYPOST_CAL_DISTANCE + OFFSET_DIST); //absolute position into a cell
    //offset dist
    move(0, OFFSET_DIST, SPYPOST_MOVE_SPEED, SPYPOST_MOVE_SPEED);
    while (hasMoveEnded() != TRUE)
    {
    }
    telemetersStop();
    motorsDriverSleep(ON);

    ssd1306ClearScreen(MAIN_AREA);
}

void spyPostPrintProfile(uint32_t x, uint32_t y, uint32_t *sample, uint32_t center_x_distance)
{
    ssd1306DrawRect(0 + x, y - 2 - (SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2),
                    SPYPOST_REFERENCE_SAMPLE_WIDTH * 2 + 2,
                    SPYPOST_REFERENCE_SAMPLE_HEIGHT * 2 + 2);
    ssd1306PrintfAtLine(x, 1, &Font_3x6, "%d\n", (uint32_t) center_x_distance);
    for (uint32_t i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        for (uint32_t j = SPYPOST_REFERENCE_SAMPLE_HEIGHT; j > 0; j--)
        {
            if ((sample[i] >> (j)) & 0x01)
            {
                ssd1306DrawPixel(2 * i + x + 1, y - 1 - j * 2);
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

void spyPostKeepUsefulPart(spyPostProfileStruct *currentProfile)
{
    uint32_t i, j;
    uint32_t y_min = SPYPOST_NBITS_SAMPLING_RESOLUTION;
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
    //save the central y distance
    currentProfile->center_y_distance = y_min * SPYPOST_TELEMETER_STEPS_MEASURE_MM;
    //shift buffer (y axis)
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] = (uint32_t) currentProfile->sample[i] >> (y_min - 1); // for one pix margin
    }
    //crop buffer height
    uint32_t mask = (uint32_t) (pow(2, SPYPOST_REFERENCE_SAMPLE_HEIGHT) - 1);
    for (i = 0; i < SPYPOST_ARRAY_PROFILE_LENGTH; i++)
    {
        currentProfile->sample[i] = currentProfile->sample[i] & mask;
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
    currentProfile->center_x_distance = (i - (SPYPOST_REFERENCE_SAMPLE_WIDTH / 2)) * SPYPOST_ENCODERS_STEPS_MEASURE_MM;
    //move useful part of sample at the beginning of the buffer
    memmove(currentProfile->sample, currentProfile->sample + (i - SPYPOST_REFERENCE_SAMPLE_WIDTH),
            SPYPOST_REFERENCE_SAMPLE_WIDTH * 4);
    //Clear the rest of the buffer after move
    memset(currentProfile->sample + SPYPOST_REFERENCE_SAMPLE_WIDTH, 0,
           (SPYPOST_ARRAY_PROFILE_LENGTH - SPYPOST_REFERENCE_SAMPLE_WIDTH) * 4);
}

void spyPostSampleThicken(spyPostProfileStruct *profile, char stroke_width)
{
    uint32_t i;
    stroke_width /= 2;
    for (i = 0; i < SPYPOST_REFERENCE_SAMPLE_WIDTH; i++)
    {
        profile->sample[i] |= (uint16_t) ((profile->sample[i] >> stroke_width)
                | (uint16_t) (profile->sample[i] << stroke_width));
    }
}

uint32_t spyPostReadCalibration(void)
{
    spyPostRefTypeProfileStruct *refProfiles = NULL;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(4, 1, "USE UP OR DOWN KEYS TO SELECT", &Font_3x6);
    ssd1306Refresh();

    while (expanderJoyState() != JOY_RIGHT || refProfiles == NULL)
    {
        if (expanderJoyState() == JOY_UP)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "RIGHT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = ref_right;
        }
        if (expanderJoyState() == JOY_DOWN)
        {
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawStringAtLine(30, 0, "LEFT CALIBRATION", &Font_3x6);
            ssd1306Refresh();
            refProfiles = ref_left;
        }
        if (expanderJoyFiltered() == JOY_LEFT)
            return SPYPOST_DRIVER_E_SUCCESS;
    }
    spyPostPrintProfile(0, 64, refProfiles->wallToNoWall.sample, refProfiles->wallToNoWall.center_x_distance);
    spyPostPrintProfile(43, 64, refProfiles->singlePost.sample, refProfiles->singlePost.center_x_distance);
    spyPostPrintProfile(86, 64, refProfiles->perpendicularWall.sample,
                        refProfiles->perpendicularWall.center_x_distance);

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

    uint32_t Vmin, Vmax, Vrotate;
    Vmin = 100;
    Vmax = 100;

    moveStartCell(Vmax, Vmax);
    while (hasMoveEnded() != TRUE);
    move(0, MAIN_DIST, Vmax, Vmax); //distance with last move offset
    spyPostGetOffset(&offset);
    while (hasMoveEnded() != TRUE);
    if (offset.left_x != 0)
    {
        move(0, (OFFSET_DIST * 2.00) + offset.left_x, Vmax, Vmax);
        while (hasMoveEnded() != TRUE);
        ssd1306PrintfAtLine(90, 2, &Font_3x6, "L_x = %d", offset.left_x);
    }
    else
    {
        move(0, (OFFSET_DIST * 2.00) + offset.right_x, Vmax, Vmax);
        while (hasMoveEnded() != TRUE);
        ssd1306PrintfAtLine(90, 2, &Font_3x6, "R_x = %d", offset.right_x);
    }
    bluetoothPrintf("offset left = %d, offset right = %d\n", (uint32_t) offset.left_x, (uint32_t) offset.right_x);

    ssd1306Refresh();
    telemetersStop();
    HAL_Delay(1000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
}
