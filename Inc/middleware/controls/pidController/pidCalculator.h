/**************************************************************************/
/*!
 @file    pidCalculator.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __PIDCALCULATOR_H__
#define __PIDCALCULATOR_H__

/* dependencies*/
#include "middleware/controls/pidController/pidController.h"

/* Module Identifier */
#define PID_CALCULATOR_MODULE_ID  100

/* Error codes */
#define PID_CALCULATOR_E_SUCCESS  0
#define PID_CALCULATOR_E_ERROR    MAKE_ERROR(PID_CALCULATOR_MODULE_ID, 1)

/* Types definitions */

void accelMotor_GetStepResponse(void);
void pidEncoder_GetCriticalPoint(void);
void pidGyro_GetCriticalPoint(void);
void pidCalculator(void);

#endif
