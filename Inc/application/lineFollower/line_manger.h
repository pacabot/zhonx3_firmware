/*
 * line_manger.h
 *
 *  Created on: 13 avr. 2018
 *      Author: colin
 */

#ifndef SRC_APPLICATION_LINEFOLLOWER_LINE_MANGER_H_
#define SRC_APPLICATION_LINEFOLLOWER_LINE_MANGER_H_

#include "application/lineFollower/lineFollower.h"

#define MAX_COLORS_COUPLES 2

extern int line_speed;
extern int line_length;
extern int line_center_offset;


void lineFollower(void);
void line_sensor_linear_calib();

#endif /* SRC_APPLICATION_LINEFOLLOWER_LINE_MANGER_H_ */
