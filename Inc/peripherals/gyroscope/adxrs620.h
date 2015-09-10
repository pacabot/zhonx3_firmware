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

/* Error codes */
#define ADXRS620_DRIVER_E_SUCCESS  0
#define ADXRS620_DRIVER_E_ERROR    MAKE_ERROR(ADXRS620_DRIVER_MODULE_ID, 1)

/* Types definitions */
typedef struct
{
    uint16_t adc_value;
    uint32_t callback_cnt;
    volatile double current_angle;
}gyro_struct;

volatile gyro_struct gyro;

void   	adxrs620Init(void);
double 	adxrs620Calibrate(int nb_ech);
void 	adxrs620_IT(void);
void 	adxrs620_ADC_IT(void);
double 	GyroGetAngle(void);
void 	GyroResetAngle(void);
void 	adxrs620Test(void);
void 	adxrs620Cal(void);

#endif
