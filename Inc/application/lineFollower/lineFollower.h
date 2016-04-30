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

#define MAXSPEED 300
#define MINSPEED 100
#define SPEED_COEFF 1.00

typedef struct
{
    unsigned short int left;
    unsigned short int front;
    unsigned short int right;
    unsigned short int leftExt;
    unsigned short int rightExt;
} ground_sensors_struct;
typedef struct
{
	int left;
	int front;
	int right;
	int leftExt;
	int rightExt;
} calibrate_sensors_struct;

void lineSensorsCalibration(void);
int EstAGauche();
void lineFollower(void);
int lineFollowerStop();
int lineFollowerFigure();
void lineFollower_IT(void);
void controlLoop(void);

#endif /* LINE_FOLLOWER_H_ */
