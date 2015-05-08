/*
 * lineFollower.h
 *
 *  Created on: 5 May 2015
 *      Author: Bertrand
 */

#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

void LineTest(void);
void lineFollower_test(double LM, double FM, double RM);

/* Types definitions */
typedef struct
{
	double position;			//total distance
//	double max_speed;
//	double speed_average;
//	double accel;
//	double decel;
//	double accel_dist;
//	double decel_dist;
//	double accel_time;
//	double decel_time;
//	double accel_speed_avrg;
//	double decel_speed_avrg;
//	double accel_dist_per_loop;
//	double decel_dist_per_loop;
//	double nb_loop_accel;
//	double nb_loop_decel;
//	double nb_loop_maint;
//	double maintain_dist;
//	int   sign;
}lineFollower_struct;
extern lineFollower_struct lineFollower;

#endif /* LINE_FOLLOWER_H_ */
