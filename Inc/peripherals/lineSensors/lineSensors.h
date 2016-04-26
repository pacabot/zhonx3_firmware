/**************************************************************************/
/*!
 @file     lineSensors.h
 @author   PLF Pacabot.com
 @date     01 December 2014
 @version  0.10
 */
/**************************************************************************/
#ifndef __LINESENSORS_H__
#define __LINESENSORS_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define LINESENSORS_DRIVER_E_SUCCESS  0
#define LINESENSORS_DRIVER_E_ERROR    MAKE_ERROR(LINESENSORS_DRIVER_MODULE_ID, 1)

enum linesensorName
{
    LINESENSOR_EXT_L, LINESENSOR_L, LINESENSOR_F, LINESENSOR_R, LINESENSOR_EXT_R
};

void    lineSensorsInit(void);
void    lineSensorsStart(void);
void    lineSensorsStop(void);
double  getLineSensorAdc(enum linesensorName linesensor_name);
void    lineSensors_IT(void);
void    lineSensors_ADC_IT(ADC_HandleTypeDef *hadc);
void    lineSensorsTest(void);

#endif
