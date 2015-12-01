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

/* dependencies*/
#include "middleware/controls/pidController/pidController.h"

/* Module Identifier */
#define SPEED_CONTROL_MODULE_ID  100

/* Error codes */
#define SPEED_CONTROL_E_SUCCESS  0
#define SPEED_CONTROL_E_ERROR    MAKE_ERROR(SPEED_CONTROL_MODULE_ID, 1)

/* Types definitions */

//enum speedType { ACC, DCC, STOP, MAINTAIN };
enum speedRate { LOWSPEED = 400, MEDIUMSPEED = 800, FASTSPEED = 1000, HIGHSPEED = 2000 };

int 	speedControlInit(void);
char 	speedControlHasMoveEnded(void);
double 	speedControlGetCurrentDist(void);
double 	speedControlGetSpeedCommand(void);
double 	speedControlSetSign(double sign);
int 	speedControlLoop(void);
int 	speedCompute(void);
double 	speedProfileCompute(double distance, double max_speed, double end_speed);
double 	speedMaintainCompute(void);

#endif
