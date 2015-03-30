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

enum speedType { ACC, DCC, STOP, MAINTAIN };

typedef struct
{
	float current_distance;			//distance (mm) since the control start
	float gap_distance_per_loop;	//distance between two control loop call
	float current_distance_consign;		//distance consign for current loop
	float old_distance;				//effective distance at the previous call
    float current_speed;
	float speed_consign;
	float speed_error;
	float speed_command;
	enum speedType speedType;

    pid_control_struct speed_pid;
}speed_control_struct;

extern speed_control_struct speed_control;

int speedControlInit(void);
int speedControlLoop(void);
int speedAcc(void);
int speedDcc(void);
int speedCompute(void);

#endif
