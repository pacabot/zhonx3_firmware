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

#include "middleware/controls/motionControl/speedControl.h"
#include "peripherals/telemeters/telemeters.h"

/* Types definitions */
#define CENTER_DISTANCE 	15.0
#define SUCCES_GAP_DIST 	2.0

#define MAX_DIST_FOR_ALIGN 	160.00
#define MIN_DIST_FOR_ALIGN 	3.00

#define MAX_DIST_FOR_FOLLOW 120.00
#define MIN_DIST_FOR_FOLLOW 30.00

#define BOTH_WALL_DIST 		180.00

#define DIAG_DIST_FOR_FOLLOW 87.00

#define SLIP_TRANSLATION	100

#define MAX_ANGLE_ERROR		30.00	//Degres
#define MAX_FOLLOW_ERROR	50.00	//Millimeter

#define DEADZONE_DIST		100.00	//Distance between the start of the cell and doubt area
#define DEADZONE			40.00	//doubt area

typedef struct
{
	int sign;
	int active_state;
}wall_follow_params_struct;

extern wall_follow_params_struct wall_follow_params;

typedef struct
{
	double follow_error;
	double follow_command;
	char succes;
    pid_control_struct follow_pid;
}wall_follow_control_struct;

extern wall_follow_control_struct wall_follow_control;
extern wall_follow_params_struct  wall_follow_params;

char isDeadZone(void);
int wallFollowControlInit(void);
int wallFollowControlLoop(void);
double wallFollow(telemeterStruct * telemeter);

#endif
