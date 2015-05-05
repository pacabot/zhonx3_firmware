/*
 * calibration.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

#include "application/solverMaze/solverMaze.h"

#ifndef WALL_SENSORS_H_
#define WALL_SENSORS_H_

#define WALL_SENSORS_E_SUCCESS  0
#define WALL_SENSORS_E_ERROR    MAKE_ERROR(WALL_SENSORS_MODULE_ID, 1)

#define NUMBER_OF_CELL 		100
#define DISTANCE_MEASURED	200
#if DISTANCE_MEASURED%NUMBER_OF_CELL!=0
#error you must put a multiple of NUMBER_OF_CELL in DISTANCE_MEASURED
#endif

#define NUMBER_OF_MILLIMETER_BY_LOOP DISTANCE_MEASURED/NUMBER_OF_CELL
#define NUMBER_OF_MEASURE_BY_STEP 5000

#define DISTANCE_WALL_FRONT	CELL_LENGTH
#define DISTANCE_WALL_DIAG	(CELL_LENGTH*3)/4

int wallSensorsCalibration();
int getTelemeterValueWithoutAmbientLight (int *value_front_left, int *value_front_right, int *value_diag_left, int *value_diag_right, int number_of_measure);
int getTelemetersDistance (float *distance_front_left, float *distance_front_right, float *distance_diag_left, float *distance_diag_right, int *precision);
walls getWallsPosition();
void testWallsSensors();

#endif /* WALL_SENSORS_H_ */
