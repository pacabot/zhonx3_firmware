/*
 * calibration.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

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

// select debug level or comment debug
#define DEBUG_WALL_SENSOR 	2


int wallSensorsCalibration();
int getTelemeterValueWithoutAmbientLight (int *value_front_left, int *value_front_right, int *value_diag_left, int *value_diag_right, int number_of_measure);
int getTelemetersDistance (float *distance_front_left, float *distance_front_right, float *distance_diag_left, float *distance_diag_right, int *precision);
#endif /* WALL_SENSORS_H_ */
