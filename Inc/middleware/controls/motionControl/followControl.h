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
#define CENTER_DISTANCE 50.0
#define SUCCES_GAP_DIST 3.0

enum follow_type_enum {ALIGN_FRONT, FOLLOW_WALL};

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
	char succes;
	enum follow_type_enum follow_type;
    pid_control_struct follow_pid;
}follow_control_struct;

extern follow_control_struct follow_control;

int followControlInit(void);
int followControlLoop(void);

#endif
