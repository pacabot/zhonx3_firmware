/**************************************************************************/
/*!
    @file    speedControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __SPEEDCONTROL_H__
#define __SPEEDCONTROL_H__

/* Module Identifier */
#define SPEED_CONTROL_MODULE_ID  100

/* Error codes */
#define SPEED_CONTROL_E_SUCCESS  0
#define SPEED_CONTROL_E_ERROR    MAKE_ERROR(SPEED_CONTROL_MODULE_ID, 1)

/* Types definitions */

//enum speedType { ACC, DCC, STOP, MAINTAIN };
enum speedRate { LOWSPEED = 400, MEDIUMSPEED = 800, FASTSPEED = 1000, HIGHSPEED = 2000 };

typedef struct
{
	double distance_consign;			//total distance
	double max_speed;
	double initial_speed;
    double end_speed;
	double accel;
	double decel;
	double accel_dist;
	double decel_dist;
	double accel_dist_per_loop;
	double decel_dist_per_loop;
	double nb_loop_accel;
	double nb_loop_decel;
	double nb_loop_maint;
	double maintain_dist;
	double sign;
}speed_params_struct;

extern speed_params_struct speed_params;

typedef struct
{
	double current_distance;			//distance (mm) since the control start
	double gap_distance_per_loop;	//distance between two control loop call
	double current_distance_consign;		//distance consign for current loop
	double old_distance;				//effective distance at the previous call
    double current_speed;
	double speed_consign;
	double speed_error;
	double speed_command;
	char end_control;

    pid_control_struct speed_pid;
}speed_control_struct;

extern speed_control_struct speed_control;

int speedControlInit(void);
int speedControlLoop(void);
int speedAcc(void);
int speedDcc(void);
int speedCompute(void);
float speedProfileCompute(float distance);

#endif
