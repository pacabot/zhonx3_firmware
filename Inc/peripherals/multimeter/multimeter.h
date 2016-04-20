/**************************************************************************/
/*!
 @file     multimeter.h
 @author   PLF Pacabot.com
 @date     01 January 2015
 @version  0.10
 */
/**************************************************************************/
#ifndef __MULTIMETER_H__
#define __MULTIMETER_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MULTIMETER_DRIVER_E_SUCCESS  0
#define MULTIMETER_DRIVER_E_ERROR    MAKE_ERROR(MULTIMETER_DRIVER_MODULE_ID, 1)

void mulimeterInit(void);
void multimeter_IT(void);
void multimeter_ADC_IT(void);
float multimeterGetBatVoltage(void);
float multimeterSTM32Temp(void);
float multimeterGyroTemp(void);
void mulimeterTest(void);

#endif
