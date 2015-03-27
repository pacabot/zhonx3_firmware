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

/* Types definitions */
typedef struct
{
	float current_angle;
	float gap_orientation_per_loop;
	float old_orientation;
    float current_angular_speed;
	float rotation_consigne;
	float rotation_error;
	float position_command;

    pid_control_struct position_pid;
}position_control_struct;


extern position_control_struct position_control;

int positionControlInit(void);
int positionControlLoop(void);

#endif
