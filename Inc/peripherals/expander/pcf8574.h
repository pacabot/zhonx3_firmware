/**************************************************************************/
/*!
    @file     PCF8574.h
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for expander PCF8574
 */
/**************************************************************************/
#ifndef __PCF8574_H__
#define __PCF8574_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define PCF8574_DRIVER_E_SUCCESS  0
#define PCF8574_DRIVER_E_ERROR    MAKE_ERROR(PCF8574_DRIVER_MODULE_ID, 1)

#define JOY_UP 		1
#define JOY_DOWN 	2
#define JOY_LEFT 	3
#define JOY_RIGHT 	4
#define JOY_SEVERAL 255

extern unsigned int joy_activ_old_time;

void expanderInit(void);

void expanderLedState(char led, char val);
char expanderJoyState(void);

void expanderSetbit(char pin, char val);
char expanderGetbit(char pin);

void joystickTest(void);
void antiBounceJoystick(void);
char antiBounceJoystick2(char joystick);
char expanderJoyFiltered(void);

#endif
