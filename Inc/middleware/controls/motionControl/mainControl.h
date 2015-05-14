/**************************************************************************/
/*!
    @file    mainControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __MAINCONTROL_H__
#define __MAINCONTROL_H__

#include "config/config.h"
#include "middleware/controls/motionControl/speedControl.h"

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MAIN_CONTROL_E_SUCCESS  0
#define MAIN_CONTROL_E_ERROR    MAKE_ERROR(MAIN_CONTROL_MODULE_ID, 1)

/* Types definitions */
enum rotation_type_enum {CW, CCW};

extern const double ROTATION_DIAMETER;

typedef struct
{
	char follow_state;
	char position_state;
	char speed_state;
}control_params_struct;

extern control_params_struct control_params;

int mainControlInit(void);
int mainControlLoop(void);
/**
 * @brief compute and start a new movement
 *
 * This function outputs all params for pids controller (speedControl and positionControl).
 *
 * @param
 *
 * angle => angle of rotation in degres (0° for strait move)
 * radius_or_distance => radius in mm of rotate if angle > 0 or distance in mm if angle = 0°
 * speed_rate => speed ratio in percent of max value
 *
 * @retval HAL status
 */
int  move(float angle, float radius_or_distance, float max_speed, float end_speed);
char isEndMove(void);
int  frontCal(float max_speed);
int  rotate180WithCal(enum rotation_type_enum rotation_type, float max_speed, float end_speed);
int  rotate90WithCal(enum rotation_type_enum rotation_type, float max_speed, float end_speed);
int  moveOneCell(float max_speed, float end_speed);
int  moveHalfCell(float max_speed, float end_speed);
int  mouveRotateCW90(float max_speed, float end_speed);
int  mouveRotateCCW90(float max_speed, float end_speed);
void mainControlTest(void);
void followWallTest(void);
void followLineTest(void);
void rotateTest(void);
void curveRotate(void);

#endif // __MAINCONTROL_H
