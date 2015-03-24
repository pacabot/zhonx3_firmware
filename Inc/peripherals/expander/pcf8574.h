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
#define PCF8574_DRIVER_MODULE_ID  6

/* Error codes */
#define PCF8574_DRIVER_E_SUCCESS  0
#define PCF8574_DRIVER_E_ERROR    MAKE_ERROR(PCF8574_DRIVER_MODULE_ID, 1)

#define UP 		1
#define DOWN 	2
#define LEFT 	3
#define RIGHT 	4
#define SEVERAL 255

void expanderLedState(char led, char val);
char expanderJoyState(void);

void expanderSetbit(char pin, char val);
char expanderGetbit(char pin);
void expanderReset(void);

void joystickTest(void);
void antiBounceJoystick(void);
char antiBounceJoystick2(char joystick);
char expanderJoyFiltered(void);

#endif
