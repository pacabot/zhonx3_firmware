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

#include "middleware/controls/motionControl/speedControl.h"

/* Types definitions */
#define CENTER_DISTANCE 	15.0
#define SUCCES_GAP_DIST 	3.0

#define MAX_DIST_FOR_ALIGN 	160.00
#define MIN_DIST_FOR_ALIGN 	3.00

#define MAX_DIST_FOR_FOLLOW 120.00
#define MIN_DIST_FOR_FOLLOW 30.00

#define BOTH_WALL_DIST 		180.00

#define DIAG_DIST_FOR_FOLLOW 86.00

typedef struct
{
	int sign;
	int active_state;
}line_follow_params_struct;

extern line_follow_params_struct line_follow_params;

typedef struct
{
	double line_follow_error;
	double line_follow_command;
	char succes;
    pid_control_struct line_follow_pid;
}line_follow_control_struct;

extern line_follow_control_struct line_follow_control;

int lineFollowControlInit(void);
int lineFollowControlLoop(void);

#endif
