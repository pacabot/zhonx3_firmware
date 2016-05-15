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
enum mainControlMoveType
{
    STRAIGHT, CURVE, ROTATE_IN_PLACE
};
enum mainControlFollowType
{
    LINE_FOLLOW, WALL_FOLLOW, NO_FOLLOW
};

typedef struct
{
    enum mainControlMoveType moveType;
    walls cellState;
} move_params_struct;

extern double ROTATION_DIAMETER;

int                             mainControlInit(void);
int                             mainControlStopPidLoop(void);
int                             mainControlSartPidLoop(void);
int                             mainControl_IT(void);
int                             mainControlSetFollowType(enum mainControlFollowType followType);
enum mainControlFollowType      mainControlGetFollowType(void);
enum mainControlMoveType        mainControlGetMoveType(void);
int                             mainControlSetMoveType(enum mainControlMoveType moveType);
char                            hasMoveEnded(void);

#endif // __MAINCONTROL_H
