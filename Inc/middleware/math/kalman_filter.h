/*
 * kalman_filter.h
 *
 *  Created on: 6 janv. 2016
 *      Author: Colin
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
typedef struct
{
	float p;
	float q;
	float r;
	float pc;
	float k;
	float xp;
	float xe;
	float zp;
}kalman_filter_params;

void Kalman_filter_init (kalman_filter_params *params, float p, float q, float r);
float Kalman_filter (kalman_filter_params *params, float value_to_filtre);
void Kalman_filter_reset_to_value (kalman_filter_params *params, float value);
#endif /* KALMAN_FILTER_H_ */
