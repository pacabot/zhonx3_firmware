/**************************************************************************/
/*!
    @file     ADXRS620.h
    @author   PLF Pacabot.com
    @date     03 August 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __ADXRS620_H__
#define __ADXRS620_H__

#include "config/config.h"

void ADXRS620_Init(void);
void ADXRS620_Calibrate(int nb_ech);
void ADXRS620_IT(void);
void Debug_ADXRS620(void);

#endif
