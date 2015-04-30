/**************************************************************************/
/*!
    @file    followControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __FOLLOWCONTROL_H__
#define __FOLLOWCONTROL_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define FOLLOW_CONTROL_E_SUCCESS  0
#define FOLLOW_CONTROL_E_ERROR    MAKE_ERROR(FOLLOW_CONTROL_MODULE_ID, 1)

#include "middleware/controls/motionControl/speedControl.h"

/* Types definitions */
typedef struct
{
	int sign;
	int active_state;
}follow_params_struct;

extern follow_params_struct follow_params;

typedef struct
{
	double follow_error;
	double follow_command;
    pid_control_struct follow_pid;
}follow_control_struct;

extern follow_control_struct follow_control;

int followControlInit(void);
int followControlLoop(void);

#endif
