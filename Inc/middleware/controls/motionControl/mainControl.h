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
#define MAIN_CONTROL_MODULE_ID  101

/* Error codes */
#define MAIN_CONTROL_E_SUCCESS  0
#define MAIN_CONTROL_E_ERROR    MAKE_ERROR(MAIN_CONTROL_MODULE_ID, 1)

/* Types definitions */

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
int move(float angle, float radius_or_distance, enum speedRate speed_rate, float end_speed);
void mainControlTest(void);

#endif // __MAINCONTROL_H
