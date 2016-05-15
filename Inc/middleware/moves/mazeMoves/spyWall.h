/**************************************************************************/
/*!
 @file    SPYWALL.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __SPYWALL_H__
#define __SPYWALL_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define SPYWALL_E_SUCCESS  0
#define SPYWALL_E_ERROR    MAKE_ERROR(SPYWALL_MODULE_ID, 1)

//#define DEBUG_SPYWALL

typedef struct
{
    double calib_value;
} spyWallCalibStruct;

typedef struct
{
    int32_t front_dist;
} spyWallGetOffsetsStruct;

int  spyWallGetFrontDist(spyWallGetOffsetsStruct *offset);
void spyWallFrontDistCal(void);
void spyWallFrontTest(void);

#endif //SPYWALL
