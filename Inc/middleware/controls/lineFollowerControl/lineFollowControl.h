/**************************************************************************/
/*!
 @file    followControl.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __LINEFOLLOWCONTROL_H__
#define __LINEFOLLOWCONTROL_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define LINE_FOLLOW_CONTROL_E_SUCCESS  0
#define LINE_FOLLOW_CONTROL_E_ERROR    MAKE_ERROR(LINE_FOLLOW_CONTROL_MODULE_ID, 1)

#include <middleware/controls/mainControl/speedControl.h>

/* Types definitions */

typedef struct
{
    int sign;
    int active_state;
} line_follow_params_struct;

extern line_follow_params_struct line_follow_params;

typedef struct
{
    double line_follow_error;
    double line_follow_command;
    char succes;
    pid_control_struct line_follow_pid;
} line_follow_control_struct;

extern line_follow_control_struct line_follow_control;

extern int Line_follower_KP;
extern int Line_follower_KD;

int lineFollowControlInit(void);
int lineFollowControlLoop(void);

#endif
