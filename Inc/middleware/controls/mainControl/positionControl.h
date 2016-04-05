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

/* Types definitions */

#define GYRO_ENCODER_RATIO 	((1.00/180.00) * PI * ROTATION_DIAMETER)

enum enablePositionCtrl
{
    NO_POSITION_CTRL, POSITION_CTRL
};

enum positionType
{
    ENCODERS, GYRO
};

int positionControlInit(void);
char positionControlHasMoveEnded(void);
double positionControlGetCurrentAngle(void);
double positionControlGetPositionCommand(void);
char positionControlSetPositionType(enum positionType position_type);
char positionControlEnablePositionCtrl(enum enablePositionCtrl enable_position_ctrl);
int positionControlLoop(void);
double positionProfileCompute(double angle, double loop_time, double max_speed);

#endif
