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

/* Module Identifier */
#define PID_CALCULATOR_MODULE_ID  100

/* Error codes */
#define PID_CALCULATOR_E_SUCCESS  0
#define PID_CALCULATOR_E_ERROR    MAKE_ERROR(PID_CALCULATOR_MODULE_ID, 1)

/* Types definitions */
typedef struct
{
    double kp;
    double ki;
    double kd;
} pid_coefs;

void pidEncoder_GetCriticalPoint(void);
void pidGyro_GetCriticalPoint(void);
void pidTelemeters_GetCriticalPoint(void);

#endif
