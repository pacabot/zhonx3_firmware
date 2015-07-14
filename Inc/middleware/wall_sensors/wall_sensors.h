/*
 * calibration.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

#include "application/solverMaze/solverMaze.h"
#include "peripherals/telemeters/telemeters.h"

#ifndef WALL_SENSORS_H_
#define WALL_SENSORS_H_

#define WALL_SENSORS_E_SUCCESS  0
#define WALL_SENSORS_E_ERROR    MAKE_ERROR(WALL_SENSORS_MODULE_ID, 1)

#if ((DISTANCE_MEASURED) % (NUMBER_OF_CELL)) != 0
#error you must put a multiple of NUMBER_OF_CELL in DISTANCE_MEASURED
#endif

//#define NUMBER_OF_MEASURE_BY_STEP 5000

#define DISTANCE_FIRST_WALL_FRONT	190.00
#define DISTANCE_SEGOND_WALL_FRONT	200.00
#define DISTANCE_WALL_DIAG			160.00

void wallSensorInit(void);
int wallSensorsCalibration();
//int getTelemetersDistance ( telemetersStruct *telemeters);
walls getCellState();
void testWallsSensors();

telemetersStruct * getDistance_ptr(void);

#endif /* WALL_SENSORS_H_ */
