/*
 * spyPost.h
 *
 *  Created on: 19 mars 2016
 *      Author: zhonx
 */

#ifndef _SPYPOST_H_
#define _SPYPOST_H_

#include "config/config.h"

#define SPYPOST_ENCODERS_STEPS_MEASURE_MM      1
#define SPYPOST_CAL_DISTANCE                   (uint32_t)CELL_LENGTH
#define SPYPOST_ARRAY_PROFILE_LENGTH           ((SPYPOST_CAL_DISTANCE)/SPYPOST_ENCODERS_STEPS_MEASURE_MM)

/* Types definitions */
typedef enum
{
    WALL_TO_NO_WALL = 1, SINGLE_POST = 2, PERPENDICULAR_WALL = 3
} spyPostEnum;

typedef struct
{
    int32_t left_x;
    int32_t right_x;
    int32_t left_y;
    int32_t right_y;
    spyPostEnum left_spyPostType;
    spyPostEnum right_spyPostType;
} spyPostGetOffsetsStruct;

typedef struct
{
    uint64_t sample[SPYPOST_ARRAY_PROFILE_LENGTH + 1];
    int32_t center_x_distance;
    int32_t center_y_distance;
} spyPostProfileStruct;

typedef struct
{
    spyPostProfileStruct wallToNoWall;
    spyPostProfileStruct singlePost;
    spyPostProfileStruct perpendicularWall;
    unsigned int         initializer;
} spyPostRefProfileStruct;

#define SPYPOST_DRIVER_E_SUCCESS        0
#define SPYPOST_DRIVER_E_ERROR          MAKE_ERROR(SPYPOST_DRIVER_MODULE_ID, 1)
#define SPYPOST_DRIVER_E_NOT_CALIBRATED MAKE_ERROR(SPYPOST_DRIVER_MODULE_ID, 2)

//#define DEBUG_SPYPOST
//#define ENABLE_SINGLE_POST

int spyPostInit(void);
uint32_t spyPostGetOffset(spyPostGetOffsetsStruct *offset);
uint32_t spyPostCalibration(void);
uint32_t spyPostReadCalibration(void);
void spyPostTest();

#endif /* _SPYPOST_H_ */
