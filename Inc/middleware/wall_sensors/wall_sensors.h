/*
 *  wall_sensors.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

#ifndef WALL_SENSORS_H_
#define WALL_SENSORS_H_

#define WALL_SENSORS_E_SUCCESS  0
#define WALL_SENSORS_E_ERROR    MAKE_ERROR(WALL_SENSORS_MODULE_ID, 1)

/* Types definitions */
typedef enum
{
    LEFT_WALL = 1, RIGHT_WALL = 2, FRONT_WALL = 3
} wallSelectorEnum;

//#define NUMBER_OF_MEASURE_BY_STEP 5000

#define DISTANCE_WALL_DIAG		    (140.00)
#define DISTANCE_WALL_FRONT		    (210.00)
//#define DISTANCE_SEGOND_WALL_FRONT	(220.00)

/* Types definitions */

typedef struct
{
    char next_front;
    char front;
    char left;
    char right;
} walls;

extern walls cell_state;

int  getWallPresence(wallSelectorEnum wallSelector);
void testWallsSensors();
void testPostSensors();

#endif /* WALL_SENSORS_H_ */
