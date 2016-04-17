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

#define MAX_FRONT_DIST_ERROR OFFSET_DIST * 2.00

//#define DEBUG_REPOSITION

enum telemeters_used
{
    NO_SIDE, ALL_SIDE, LEFT_SIDE, RIGHT_SIDE
};

typedef struct
{
    int32_t front_dist;
} repositionGetOffsetsStruct;

void repositionSetInitialPosition(double initial_position);
void repositionResetTelemeterUsed(void);
enum telemeters_used repositionGetTelemeterUsed(void);
int  repositionGetFrontDist(repositionGetOffsetsStruct *offset);
void repositionGetFrontDistCal(void);
int  frontCal(float max_speed);

#endif
