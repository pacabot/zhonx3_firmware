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

#if ((DISTANCE_MEASURED) % (TELEMETER_PROFILE_ARRAY_LENGTH)) != 0
#error you must put a multiple of NUMBER_OF_CELL in DISTANCE_MEASURED
#endif

/* Types definitions */
typedef enum
{
    LEFT_WALL = 1, RIGHT_WALL = 2, FRONT_WALL = 3
} wallSelectorEnum;

//#define NUMBER_OF_MEASURE_BY_STEP 5000

#define DISTANCE_WALL_DIAG		140.00
#define DISTANCE_WALL_FRONT		300.00
#define DISTANCE_SEGOND_WALL_FRONT	200.00

/* Types definitions */

typedef struct
{
    char next_front;
    char front;
    char left;
    char right;
} walls;

extern walls cell_state;

char getWallPresence(wallSelectorEnum wallSelector);

#endif /* WALL_SENSORS_H_ */
