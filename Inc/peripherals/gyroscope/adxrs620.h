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

/* Module Identifier */
#include "config/module_id.h"

// Structure that will contain the calibration values for the Gyroscope
typedef struct
{
    double calib_value;
} gyro_calib_struct;

/* Error codes */
#define ADXRS620_DRIVER_E_SUCCESS  0
#define ADXRS620_DRIVER_E_ERROR    MAKE_ERROR(ADXRS620_DRIVER_MODULE_ID, 1)

void adxrs620Init(void);
double adxrs620Calibrate(int nb_ech);
void adxrs620_IT(void);
void adxrs620_ADC_IT(void);
double gyroGetAngle(void);
void gyroResetAngle(void);
void adxrs620Cal(void);
void adxrs620Test(void);

#endif
