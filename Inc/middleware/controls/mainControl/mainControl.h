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

#include <middleware/controls/mainControl/speedControl.h>
#include "config/config.h"
#include "middleware/wall_sensors/wall_sensors.h"

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MAIN_CONTROL_E_SUCCESS  0
#define MAIN_CONTROL_E_ERROR    MAKE_ERROR(MAIN_CONTROL_MODULE_ID, 1)

//#define DEBUG_MAIN_CONTROL

/* Types definitions */
enum rotationTypeEnum
{
    CW, CCW
};
enum mainControlWallFollowType
{
    STRAIGHT, CURVE
};
enum mainControlFollowType
{
    LINE_FOLLOW, WALL_FOLLOW, NO_FOLLOW
};

typedef struct
{
    enum mainControlWallFollowType moveType;
    walls cellState;
} move_params_struct;

extern move_params_struct move_params;

extern double ROTATION_DIAMETER;

int mainControlInit(void);
int mainControl_IT(void);
int setWallFollowControl(char isActive);
char hasMoveEnded(void);
int mainControlSetFollowType(enum mainControlFollowType follow_type);
enum mainControlFollowType mainControlGetFollowType(void);
enum mainControlWallFollowType mainControlGetWallFollowType(void);
double positionControlSetSign(double sign);

/**
 * @brief compute and start a new movement
 *
 * This function outputs all params for pids controller (speedControl and positionControl).
 *
 * @param
 *
 * angle => angle of rotation in degres (0° for strait move)
 * radius_or_distance => radius in mm of rotate if angle > 0 or distance in mm if angle = 0°
 * max_speed => max speed
 * end_speed => end speed for chain
 *
 * @retval status
 */
int move(double angle, double radius_or_distance, double max_speed, double end_speed);
/**
 * @brief compute and start a new straight movement
 *
 * This function outputs all params for pids controller (speedControl and positionControl).
 *
 * @param
 *
 * angle => angle of rotation in degres (0° for strait move)
 * distance => distance in mm
 * max_speed => max speed
 * end_speed => end speed for chain
 * accel     => acceleration mm/s²
 * @retval status
 */
int moveStraight(double distance, double max_speed, double end_speed, double accel);

#endif // __MAINCONTROL_H
