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
	double distance_consign;			//total distance
	double max_speed;
	double speed_average;
	double accel;
	double decel;
	double accel_dist;
	double decel_dist;
	double accel_time;
	double decel_time;
	double accel_speed_avrg;
	double decel_speed_avrg;
	double accel_dist_per_loop;
	double decel_dist_per_loop;
	double nb_loop_accel;
	double nb_loop_decel;
	double nb_loop_maint;
	double maintain_dist;
	int   sign;
}follow_params_struct;

extern follow_params_struct follow_params;

typedef struct
{
	double current_angle;
	double follow_command;
	double follow_error;
	double follow_consign;
	double current_diff_dist;
	double current_diff_dist_consign;	//differential distance (mm) since the control start
	double old_distance;				 	//effective distance at the previous call
	char  end_control;

    pid_control_struct follow_pid;
}follow_control_struct;

extern follow_control_struct follow_control;

int followControlInit(void);
int followControlLoop(void);

#endif
