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
#define TIMESBASE_DRIVER_MODULE_ID  13

/* Error codes */
#define TIMESBASE_DRIVER_E_SUCCESS  0
#define TIMESBASE_DRIVER_E_ERROR    MAKE_ERROR(TIMESBASE_DRIVER_MODULE_ID, 1)

void timesBaseInit(void);
void ledPowerBlink(unsigned int off_time, unsigned int on_time, unsigned int repeat);
void ledBlink_IT(void);
void highFreq_IT(void);
void lowFreq_IT(void);


#endif
