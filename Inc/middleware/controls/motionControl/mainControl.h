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
void straightMove(float distance, enum speedRate speed_rate);
void mainControlTest(void);

#endif // __MAINCONTROL_H
