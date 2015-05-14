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
	char   active_state;
}line_follower_struct;
extern line_follower_struct line_follower;

typedef struct
{
	double left;
	double front;
	double right;
	double leftExt;
	double rightExt
} ground_sensors_struct;

void lineTest(void);
void lineFollower_IT(void);
void asservissement(void);

#endif /* LINE_FOLLOWER_H_ */
