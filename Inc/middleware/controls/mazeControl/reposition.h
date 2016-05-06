/**************************************************************************/
/*!
 @file    reposition.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __REPOSITION_H__
#define __REPOSITION_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define REPOSITION_E_SUCCESS  0
#define REPOSITION_E_ERROR    MAKE_ERROR(REPOSITION_MODULE_ID, 1)

//#define DEBUG_REPOSITION

enum telemeters_used
{
    NO_SIDE, ALL_SIDE, LEFT_SIDE, RIGHT_SIDE
};

typedef struct
{
    double calib_value;
} reposition_calib_struct;

typedef struct
{
    int32_t front_dist;
} repositionGetOffsetsStruct;

void repositionSetInitialPosition(double initial_position);
void repositionResetTelemeterUsed(void);
enum telemeters_used repositionGetTelemeterUsed(void);
int  repositionGetFrontDist(repositionGetOffsetsStruct *offset);
void repositionFrontDistCal(void);
void repositionFrontTest(void);

#endif
