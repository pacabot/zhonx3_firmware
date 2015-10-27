/*
 * calibration.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

#include "application/solverMaze/solverMaze.h"
#include "peripherals/telemeters/telemeters.h"

#ifndef TELEMETERS_CAL_H_
#define TELEMETERS_CAL_H_

#define TELEMETERS_CAL_E_SUCCESS  0
#define TELEMETERS_CAL_E_ERROR    MAKE_ERROR(TELEMETERS_CAL_MODULE_ID, 1)

#if ((DISTANCE_MEASURED) % (NUMBER_OF_CELL)) != 0
#error you must put a multiple of NUMBER_OF_CELL in DISTANCE_MEASURED
#endif

//#define NUMBER_OF_MEASURE_BY_STEP 5000

/* Types definitions */


int   wallSensorsCalibrationFront(void);
int	  wallSensorsCalibrationDiag (void);
void  testWallsSensors();
void  testPostSensors();

#endif /* TELEMETERS_CAL_H_ */
