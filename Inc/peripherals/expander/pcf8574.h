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

#include "stdbool.h"

#define UP 		1
#define DOWN 	2
#define LEFT 	3
#define RIGHT 	4
#define SEVERAL 255

void Expander_Led_State(char led, char val);
char Expander_Joy_State(void);

void ExpanderSetbit(char pin, char val);
char ExpanderGetbit(char pin);
void ExpanderReset(void);

void Debug_Joystic(void);

#endif
