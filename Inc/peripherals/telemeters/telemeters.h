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

#define TELEMETERS_CAL_E_SUCCESS  0
#define TELEMETERS_CAL_E_ERROR    MAKE_ERROR(TELEMETERSCAL_DRIVER_MODULE_ID, 1)

#define TELEMETER_PROFILE_ARRAY_LENGTH 		250
#define MEASURED_DISTANCE	                250
#define NUMBER_OF_MILLIMETER_BY_LOOP (MEASURED_DISTANCE/TELEMETER_PROFILE_ARRAY_LENGTH)

#if ((DISTANCE_MEASURED) % (TELEMETER_PROFILE_ARRAY_LENGTH)) != 0
#error you must put a multiple of NUMBER_OF_CELL in DISTANCE_MEASURED
#endif

enum telemeterName
{
    TELEMETER_FL, TELEMETER_DL, TELEMETER_DR, TELEMETER_FR
};

typedef struct
{
    int right[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
    int left[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
} FRONT_TELEMETERS_PROFILE;

typedef struct
{
    int right[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
    int left[TELEMETER_PROFILE_ARRAY_LENGTH + 1];
} DIAG_TELEMETERS_PROFILE;

typedef struct
{
    FRONT_TELEMETERS_PROFILE front;
    DIAG_TELEMETERS_PROFILE diag;
} TELEMETERS_PROFILE;

extern TELEMETERS_PROFILE *telemeters_profile;

/* Exported functions for telemeterCal.c*/
int wallSensorsCalibrationFront(void);
int wallSensorsCalibrationDiag(void);
void testWallsSensors();
void testPostSensors();

/* Exported functions for telemeter.c */
void telemetersInit(void);
void telemetersStart(void);
void telemetersStop(void);
double getTelemeterDist(enum telemeterName telemeter_name);
int getTelemeterAvrg(enum telemeterName telemeter_name);
double getTelemeterSpeed(enum telemeterName telemeter_name);
void telemeters_IT(void);
void telemeters_DMA_IT(void);
void telemeters_ADC2_IT(void);
void telemeters_ADC3_IT(void);
void telemetersTest(void);

#endif //__TELEMETERS_H__
