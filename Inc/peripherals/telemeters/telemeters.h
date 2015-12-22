/**************************************************************************/
/*!
    @file     telemeter.h
    @author   PLF Pacabot.com
    @date     03 August 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __TELEMETERS_H__
#define __TELEMETERS_H__

/* Module Identifier */
#include "config/module_id.h"
#include "application/statistiques/statistiques.h"

/* Error codes */
#define TELEMETERS_DRIVER_E_SUCCESS  0
#define TELEMETERS_DRIVER_E_ERROR    MAKE_ERROR(TELEMETERS_DRIVER_MODULE_ID, 1)

#define TELEMETER_PROFILE_ARRAY_LENGTH 		150
#define MEASURED_DISTANCE	300
#define NUMBER_OF_MILLIMETER_BY_LOOP (MEASURED_DISTANCE/TELEMETER_PROFILE_ARRAY_LENGTH)

enum telemeterName {TELEMETER_FL, TELEMETER_DL, TELEMETER_DR, TELEMETER_FR};

extern int telemeter_FR_profile[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
extern int telemeter_FL_profile[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
extern int telemeter_DR_profile[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
extern int telemeter_DL_profile[TELEMETER_PROFILE_ARRAY_LENGTH + 1];

/* Exported functions */
void   telemetersInit(void);
void   telemetersStart(void);
void   telemetersStop(void);
double getTelemeterDist(enum telemeterName telemeter_name);
double getTelemeterAvrg(enum telemeterName telemeter_name);
double getTelemeterSpeed(enum telemeterName telemeter_name);
void   telemeters_IT(void);
void   telemeters_DMA_IT(void);
void   telemeters_ADC2_IT(void);
void   telemeters_ADC3_IT(void);
void   telemetersTest(void);

#endif //__TELEMETERS_H__
