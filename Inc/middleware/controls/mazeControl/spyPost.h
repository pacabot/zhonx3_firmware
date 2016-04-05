/*
 * spyPost.h
 *
 *  Created on: 19 mars 2016
 *      Author: zhonx
 */

#ifndef _SPYPOST_H_
#define _SPYPOST_H_

typedef struct
{
    int32_t left_x;
    int32_t right_x;
    int32_t left_y;
    int32_t right_y;
} spyPostGetOffsetsStruct;

#define SPYPOST_DRIVER_E_SUCCESS  0
#define SPYPOST_DRIVER_E_ERROR    MAKE_ERROR(SPYPOST_DRIVER_MODULE_ID, 1)

//#define DEBUG_SPYPOST

uint32_t spyPostGetOffset(spyPostGetOffsetsStruct *offset);
uint32_t spyPostCalibration(void);
uint32_t spyPostReadCalibration(void);
void spyPostTest();

#endif /* _SPYPOST_H_ */
