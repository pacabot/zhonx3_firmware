/**************************************************************************/
/*!
 @file     TimesBase.h
 @author   PLF Pacabot.com
 @date     03 August 2014
 @version  0.10
 */
/**************************************************************************/
#ifndef __TIMESBASE_H__
#define __TIMESBASE_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define TIMESBASE_DRIVER_E_SUCCESS  0
#define TIMESBASE_DRIVER_E_ERROR    MAKE_ERROR(TIMESBASE_DRIVER_MODULE_ID, 1)

//#define DEDICATED_TIMER

void timesBaseInit(void);
void timeBaseStop(void);
void ledPowerBlink(unsigned int off_time, unsigned int on_time);
void ledPowerErrorBlink(unsigned int off_time, unsigned int on_off_time, unsigned char repeat);
char timeOut(unsigned char second, int loop_nb);
void ledBlink_IT(void);

#endif
