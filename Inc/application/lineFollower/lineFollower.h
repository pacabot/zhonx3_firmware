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
}line_follower_struct;
extern line_follower_struct line_follower;

typedef struct
{
	double left;
	double front;
	double right;
} ground_sensors;

void LineTest(void);
void line_follower_test(ground_sensors, ground_sensors);

#endif /* LINE_FOLLOWER_H_ */
