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
typedef struct
{
	float current_distance;			//distance (mm) since the control start
	float gap_distance_per_loop;	//distance between two control loop call
	float old_distance;				//effective distance at the previous call
    float current_speed;
	float speed_consigne;
	float distance_consigne;
	float speed_error;
	float speed_command;

    CONTROL_DEF speed;
}speed_control_struct;

extern speed_control_struct speed_control;

int speedControl_Init(void);
int speedControlLoop(void);
int speedAcc(uint32_t initial_speed, uint32_t distance);
int speedDcc(uint32_t final_speed, uint32_t distance);
int speedMaintain(float speed);
void speedControlTest(void);

#endif
