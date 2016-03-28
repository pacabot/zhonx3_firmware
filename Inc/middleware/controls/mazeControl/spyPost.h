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
    int left_x;
    int right_x;
    int left_y;
    int right_y;
} spyPostGetOffsetsStruct;

#define SPYPOST_DRIVER_E_SUCCESS  0
#define SPYPOST_DRIVER_E_ERROR    MAKE_ERROR(SPYPOST_DRIVER_MODULE_ID, 1)

int spyPostGetOffset(spyPostGetOffsetsStruct *offset);
int spyPostCalibration(void);

#endif /* _SPYPOST_H_ */
