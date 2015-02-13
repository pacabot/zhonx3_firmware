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

/* Types definitions */
typedef struct
{
    uint16_t adc_value;
    uint16_t calibration_current_cycle;
    uint16_t calibration_nb_cycle;
    uint32_t callback_cnt;
    volatile float current_angle;
	char calibration_state;
    volatile double alfa;
    volatile double beta;
}gyro_struct;

volatile gyro_struct gyro;

void adxrs620Init(void);
void adxrs620Calibrate(int nb_ech);
void adxrs620_IT(void);
void adxrs620_ADC_IT(void);
void adxrs620Test(void);

#endif
