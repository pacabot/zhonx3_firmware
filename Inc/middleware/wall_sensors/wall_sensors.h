/*
 *  wall_sensors.h
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

/* Types definitions */
enum wallSelectorEnum {LEFT_WALL, RIGHT_WALL, FRONT_WALL};

//#define NUMBER_OF_MEASURE_BY_STEP 5000

#define DISTANCE_WALL_DIAG		120.00
#define DISTANCE_WALL_FRONT		250.00
#define DISTANCE_SEGOND_WALL_FRONT	200.00


/* Types definitions */

typedef struct
{
	char next_front;
	char front;
	char left;
	char right;
}walls;

extern walls cell_state;

char getWallPresence(enum wallSelectorEnum wallSelector);

#endif /* WALL_SENSORS_H_ */
