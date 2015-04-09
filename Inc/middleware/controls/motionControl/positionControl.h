/**************************************************************************/
/*!
    @file    positionControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __POSITIONCONTROL_H__
#define __POSITIONCONTROL_H__

/* Module Identifier */
#define POSITION_CONTROL_MODULE_ID  102

/* Error codes */
#define POSITION_CONTROL_E_SUCCESS  0
#define POSITION_CONTROL_E_ERROR    MAKE_ERROR(POSITION_CONTROL_MODULE_ID, 1)

#include "middleware/controls/motionControl/speedControl.h"

/* Types definitions */
typedef struct
{
	float distance_consign;			//total distance
	float max_speed;
	float accel;
	float decel;
	float accel_dist;
	float decel_dist;
	float accel_dist_per_loop;
	float decel_dist_per_loop;
	float nb_loop_accel;
	float nb_loop_decel;
	float nb_loop_maint;
	float maintain_dist;
}position_params_struct;

extern position_params_struct position_params;

typedef struct
{
	float current_angle;
	float position_command;
	float position_error;
	float position_consign;
	float current_diff_dist;
	float current_diff_dist_consign;	//differential distance (mm) since the control start
	float old_distance;				 	//effective distance at the previous call
	char  end_control;

    pid_control_struct position_pid;
}position_control_struct;

extern position_control_struct position_control;

int positionControlInit(void);
int positionControlLoop(void);
int rotate_in_place(float angle, enum speedRate speed_rate);
float positionProfileCompute(float distance, float time);

#endif
