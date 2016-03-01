/**************************************************************************/
/*!
    @file     powerManagment.h
    @author   PLF Pacabot.com
    @date     03 January 2016
    @version  0.10
*/
/**************************************************************************/
#ifndef __POWERMANAGMENT_H__
#define __POWERMANAGMENT_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define POWERMANAGMENT_DRIVER_E_SUCCESS  0
#define POWERMANAGMENT_DRIVER_E_ERROR    MAKE_ERROR(MPOWERMANAGMENT_DRIVER_MODULE_ID, 1)

#define GPIO_BASE	GPIO_PIN_12

void batteryGauge_IT(void);

#endif
