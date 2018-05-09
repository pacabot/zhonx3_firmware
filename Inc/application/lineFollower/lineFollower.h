/*
 * lineFollower.h
 *
 *  Created on: 5 May 2015
 *      Author: Bertrand
 */

#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include "middleware/line_sensors/line_sensors.h"

/* Types definitions */
typedef struct
{
    double position;			//total distance
    char active_state;
} line_follower_struct;

#define LEFTEXT 1
#define LEFT_ 2
#define FRONT_ 3
#define RIGHT_ 4
#define RIGHTEXT 5

extern int line_speed;
extern int line_length;

typedef struct
{
	int left;
	int front;
	int right;
	int leftExt;
	int rightExt;
} calibrate_sensors_struct;

void lineFollower(void);
/**
 * @fn void line_print_info()
 * @brief print line follower info on the display
 */
void line_print_info(void);
void test_line_sensors(void);

#endif /* LINE_FOLLOWER_H_ */
