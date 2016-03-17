/**************************************************************************/
/*!
    @file    followControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __WALLFOLLOWCONTROL_H__
#define __WALLFOLLOWCONTROL_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define WALL_FOLLOW_CONTROL_E_SUCCESS  0
#define WALL_FOLLOW_CONTROL_E_ERROR    MAKE_ERROR(WALL_FOLLOW_CONTROL_MODULE_ID, 1)


int 	wallFollowControlInit(void);
double 	wallFollowGetCommand(void);
int 	wallFollowControlLoop(void);
double 	wallFollowMaintainCompute(void);

#endif
