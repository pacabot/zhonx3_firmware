/*
 * lineFollower.h
 *
 *  Created on: 5 May 2015
 *      Author: Bertrand
 */

#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

/* Types definitions */
typedef struct
{
    double position;			//total distance
    char active_state;
} line_follower_struct;
extern line_follower_struct line_follower;

#define LEFTEXT 1
#define LEFT_ 2
#define FRONT_ 3
#define RIGHT_ 4
#define RIGHTEXT 5

extern int line_speed;
#define MINSPEED 10

//typedef struct
//{
//    unsigned short int left;
//    unsigned short int front;
//    unsigned short int right;
//    unsigned short int leftExt;
//    unsigned short int rightExt;
//} ground_sensors_struct;
typedef struct
{
	int left;
	int front;
	int right;
	int leftExt;
	int rightExt;
} calibrate_sensors_struct;

void lineSensorsCalibration(void);
void lineFollower(void);
int lineFollowerStop();
void lineFollower_IT(void);
void controlLoop(void);
void test_line_sensors();

#endif /* LINE_FOLLOWER_H_ */
