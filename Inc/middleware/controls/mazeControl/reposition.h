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

enum telemeters_used
{
    NO_SIDE, ALL_SIDE, LEFT_SIDE, RIGHT_SIDE
};

enum telemeters_used getSensorsUsedToTrackWalls(void);
double repositionGetPostDist(double offset);
int frontCal(float max_speed);

#endif
