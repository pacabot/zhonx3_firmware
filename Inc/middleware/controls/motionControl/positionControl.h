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
#include "config/module_id.h"

/* Error codes */
#define POSITION_CONTROL_E_SUCCESS  0
#define POSITION_CONTROL_E_ERROR    MAKE_ERROR(POSITION_CONTROL_MODULE_ID, 1)

#include "middleware/controls/motionControl/speedControl.h"

/* Types definitions */

#define GYRO_ENCODER_RATIO 	((1.00/180.00) * PI * ROTATION_DIAMETER)

enum position_type_enum {GYRO, ENCODERS, NO_POSITION_CTRL, POSITION_CTRL};

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
	int    sign;
}position_params_struct;

extern position_params_struct position_params;

typedef struct
{
	double current_angle;
	double position_command;
	double position_error;
	double position_consign;
	double current_diff_dist;
	double current_diff_dist_consign;	//differential distance (mm) since the control start
	double old_distance;				 	//effective distance at the previous call
	char   end_control;
	enum   position_type_enum position_type;

    pid_control_struct position_pid;
}position_control_struct;

extern position_control_struct position_control;

int positionControlInit(void);
int positionControlLoop(void);
float positionProfileCompute(float distance, float time);

#endif
