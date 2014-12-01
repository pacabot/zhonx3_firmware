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

void TimesBase_Init(void);
void Led_Power_Blink(unsigned int off_time, unsigned int on_time, unsigned int repeat);
void Led_Blink_IT(void);
void High_Freq_IT(void);
void Low_Freq_IT(void);


#endif
