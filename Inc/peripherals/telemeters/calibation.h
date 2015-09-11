/*
 * calibation.h
 *
 *  Created on: 15 août 2015
 *      Author: zhonx
 */

#ifndef CALIBATION_H_
#define CALIBATION_H_

#include "config/module_id.h"

#define CALIBATION_E_SUCCESS	0
#define CALIBATION_E_ERROR (n_error)		MAKE_ERROR(TELEMETERS_DRIVER_MODULE_ID, n_error)

#define DISTANCE_CALIBRATED 1000 // in millimeter
#define MAX_VALUE			1025
#define STEP				5
#define NUMBER_OF_CELL 		MAX_VALUE/STEP

#if MAX_VALUE % STEP != 0
#error "MAX_VALUE must be a multiple of STEP. you have to do this calculate in iterger truncated : the new value of MAX_VAL= (MAX_VAL/STEP+1)*STEP"
#endif

extern int telemeter_FR_profile[NUMBER_OF_CELL];
extern int telemeter_FL_profile[NUMBER_OF_CELL];
extern int telemeter_DR_profile[NUMBER_OF_CELL];
extern int telemeter_DL_profile[NUMBER_OF_CELL];

#endif /* CALIBATION_H_ */
