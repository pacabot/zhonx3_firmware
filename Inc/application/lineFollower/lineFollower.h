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

void lineTest(void);
void lineFollowerTest(double CoefLeft, double CoefFront, double CoefRight);
void lineFollower_IT(void);

#endif /* LINE_FOLLOWER_H_ */
