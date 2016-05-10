/**************************************************************************/
/*!
 @file     motors.h
 @author   PLF Pacabot.com
 @date     01 December 2014
 @version  0.10
 */
/**************************************************************************/
#ifndef __MOTORS_H__
#define __MOTORS_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MOTORS_DRIVER_E_SUCCESS  0
#define MOTORS_DRIVER_E_ERROR    MAKE_ERROR(MOTORS_DRIVER_MODULE_ID, 1)

enum motorName
{
    MOTOR_L, MOTOR_R
};

void motorsInit(void);
int motorGetFault(void);
void motorsDriverSleep(int isOn);
int motorSet_DF(enum motorName motor_name, int pwm);
int motorSet_DS(enum motorName motor_name, int pwm);
void motorsBrake(void);
void motorsTest(void);

#endif
